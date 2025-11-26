import asyncio
import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from gnss_status_msg.msg import GnssDiagnostic
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from state_controller_msg.msg import State
from tf2_ros import Buffer, TransformException, TransformListener
from websockets import WebSocketServerProtocol

from logger_interfaces.srv import (
    GetLoggingMode,
    GetLoggingStatus,
    SetLoggingMode,
    ToggleLogging,
)

from .base_ws_node import BaseWsNode


class DataWsNode(BaseWsNode):
    def __init__(self):
        super().__init__("hydroball_data_websocket", port=9002)
        qos = QoSProfile(depth=10)
        self.state_sub = self.create_subscription(State, "state", self.on_state, qos)
        self.gnss_status_sub = self.create_subscription(
            GnssDiagnostic, "gnss_status", self.on_gnss_status, qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.get_logging_status = self.create_client(GetLoggingStatus, "get_logging_status")
        self.toggle_logging = self.create_client(ToggleLogging, "toggle_logging")
        self.get_logging_mode = self.create_client(GetLoggingMode, "get_logging_mode")
        self.set_logging_mode = self.create_client(SetLoggingMode, "set_logging_mode")

        self.last_timestamp_us = 0
        self.latest_gnss_status: Optional[GnssDiagnostic] = None

    def wait_for_clients(self):
        for client in (
            self.get_logging_status,
            self.toggle_logging,
            self.get_logging_mode,
            self.set_logging_mode,
        ):
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"Service {client.srv_name} unavailable")

    async def handle_message(self, websocket: WebSocketServerProtocol, message: str):
        try:
            doc = json.loads(message)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid json: {message}")
            return

        cmd = doc.get("command")
        if cmd == "getLoggingInfo":
            is_logging = self._get_recording_status()
            mode = self._get_logging_mode()
            await self._send_recording_info(websocket, is_logging, mode)
        elif cmd == "startLogging":
            self._toggle_logging(True, websocket)
        elif cmd == "stopLogging":
            self._toggle_logging(False, websocket)
        else:
            self.get_logger().error("No command found or unknown command")

    async def on_connect(self, websocket: WebSocketServerProtocol):
        # On new client, send latest gnss status and logging info if present
        if self.latest_gnss_status:
            payload = self._gnss_status_payload(self.latest_gnss_status)
            await websocket.send(payload)
        is_logging = self._get_recording_status()
        mode = self._get_logging_mode()
        await self._send_recording_info(websocket, is_logging, mode)

    async def on_disconnect(self, websocket: WebSocketServerProtocol):
        return

    def on_gnss_status(self, msg: GnssDiagnostic):
        self.latest_gnss_status = msg
        payload = self._gnss_status_payload(msg)
        self.broadcast(payload)

    def _gnss_status_payload(self, msg: GnssDiagnostic) -> str:
        doc = {
            "type": "gnss_status",
            "fix_type": int(msg.fix_type),
            "diff_soln": bool(msg.diff_soln),
            "carr_soln": int(msg.carr_soln),
            "num_sv": int(msg.num_sv),
            "h_acc": float(msg.horizontal_accuracy),
            "v_acc": float(msg.vertical_accuracy),
        }
        return json.dumps(doc)

    def _toggle_logging(self, enabled: bool, websocket: Optional[WebSocketServerProtocol]):
        req = ToggleLogging.Request()
        req.logging_enabled = enabled
        future = self.toggle_logging.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result():
            status = future.result().logging_status
            mode = self._get_logging_mode()
            if websocket:
                asyncio.run_coroutine_threadsafe(
                    self._send_recording_info(websocket, status, mode), self.loop
                )
            else:
                # broadcast to all when no specific websocket
                asyncio.run_coroutine_threadsafe(
                    self._broadcast_recording_info(status, mode), self.loop
                )
        else:
            self.get_logger().error("Error while calling ToggleLogging service")

    def _get_recording_status(self) -> bool:
        req = GetLoggingStatus.Request()
        future = self.get_logging_status.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result():
            return future.result().status
        self.get_logger().error("Error while calling GetLoggingStatus service")
        return False

    def _get_logging_mode(self) -> int:
        req = GetLoggingMode.Request()
        future = self.get_logging_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result():
            return future.result().logging_mode
        self.get_logger().error("Error while calling GetLoggingMode service")
        return 1

    async def _send_recording_info(self, ws: WebSocketServerProtocol, is_recording: bool, mode: int):
        doc = {
            "recordingInfo": {"status": bool(is_recording)},
            "loggingMode": {"the_mode_is": int(mode)},
        }
        await ws.send(json.dumps(doc))

    def on_state(self, state: State):
        timestamp_us = state.stamp.sec * 1_000_000 + state.stamp.nanosec // 1_000
        if self.last_timestamp_us and (timestamp_us - self.last_timestamp_us) <= 200_000:
            return
        self.last_timestamp_us = timestamp_us

        try:
            payload = self._state_to_json(state)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert state: {exc}")
            return

        is_logging = self._get_recording_status()
        mode = self._get_logging_mode()
        if mode == 1 and not is_logging:
            self._toggle_logging(True, None)
            is_logging = self._get_recording_status()
            mode = self._get_logging_mode()

        self.broadcast(payload)
        # Broadcast recording info along with telemetry
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._broadcast_recording_info(is_logging, mode), self.loop
            )

    async def _broadcast_recording_info(self, is_logging: bool, mode: int):
        doc = {
            "recordingInfo": {"status": bool(is_logging)},
            "loggingMode": {"the_mode_is": int(mode)},
        }
        payload = json.dumps(doc)
        await asyncio.gather(*(ws.send(payload) for ws in list(self.connections)))

    def _state_to_json(self, state: State) -> str:
        telemetry = {}

        if not state.position.header.seq and state.position.status.status < 0:
            telemetry["position"] = []
            telemetry["gnssFix"] = int(state.position.status.status)
        else:
            telemetry["position"] = [float(state.position.longitude), float(state.position.latitude)]
            telemetry["gnssFix"] = int(state.position.status.status)

        if not state.imu.header.seq:
            telemetry["attitude"] = []
        else:
            attitude = self._compute_attitude(state.imu)
            telemetry["attitude"] = list(attitude) if attitude else []

        if not state.depth.header.seq:
            telemetry["depth"] = []
        else:
            telemetry["depth"] = [float(state.depth.point.z)]

        if not state.vitals.header.seq:
            telemetry["vitals"] = []
        else:
            vit = state.vitals
            telemetry["vitals"] = [
                int(vit.cputemp),
                int(vit.cpuload),
                int(vit.freeram),
                int(vit.freehdd),
                int(vit.uptime),
                int(vit.temperature),
                float(vit.voltage),
                int(vit.humidity),
                vit.ledstate,
            ]
            telemetry["status"] = vit.status

        return json.dumps({"telemetry": telemetry})

    def _compute_attitude(self, imu: Imu):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                "base_link", "imu", Time()
            )
            rot = tf.transform.rotation
            # Apply transform to IMU orientation: combine quaternions
            # tf2 equivalent: q_result = tf * imu
            q_tf = (
                rot.x,
                rot.y,
                rot.z,
                rot.w,
            )
            q_imu = (
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            )
            q_combined = self._quat_multiply(q_tf, q_imu)
            roll, pitch, yaw = self._quat_to_rpy(q_combined)
            heading = 90.0 - math.degrees(yaw)
            if heading < 0:
                heading += 360.0
            return heading, math.degrees(pitch), math.degrees(roll)
        except TransformException as ex:
            self.get_logger().warn(f"IMU transform missing: {ex}")
            return None

    @staticmethod
    def _quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quat_to_rpy(q):
        x, y, z, w = q
        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)
    node = DataWsNode()
    node.wait_for_clients()
    node.spin()


if __name__ == "__main__":
    main()
