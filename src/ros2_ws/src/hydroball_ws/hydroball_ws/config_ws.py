import asyncio
import json
import math
import os
import subprocess
import threading
from pathlib import Path
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from setting_msg.msg import Setting
from setting_msg.srv import ConfigurationService, ImuOffsetService
from state_controller_msg.srv import GetStateService
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
from websockets import WebSocketServerProtocol

from .base_ws_node import BaseWsNode


def trim_spaces(s: str) -> str:
    return s.strip()


class ConfigWsNode(BaseWsNode):
    """
    ROS2 port of hydroball_config_websocket mirroring ROS1 behaviour:
    - Serve websocket commands getConfiguration/saveConfiguration/zeroImu
    - Broadcast configuration topic + TF offsets
    - Provide get_configuration and zero_imu_offsets services
    - Update hotspot SSID when changed
    """

    def __init__(self):
        super().__init__("hydroball_config_websocket", port=9004)
        self.config_path = Path(
            self.declare_parameter("config_path", "/opt/Poseidon/config.txt").value
        )
        self.configuration: Dict[str, str] = {}
        self.config_pub = self.create_publisher(Setting, "configuration", 10)
        self.get_state_client = self.create_client(GetStateService, "get_state")
        self.create_service(ConfigurationService, "get_configuration", self.handle_get_configuration)
        self.create_service(ImuOffsetService, "zero_imu_offsets", self.handle_zero_imu)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self._config_lock = threading.Lock()
        self.read_configuration_from_file()
        self.broadcast_configuration()
        self.broadcast_imu_transform()

    # --- File I/O ---
    def read_configuration_from_file(self):
        if not self.config_path.exists():
            self.get_logger().error(f"Cannot open file {self.config_path}")
            return
        with self._config_lock:
            self.configuration.clear()
            with self.config_path.open() as in_file:
                for line in in_file:
                    line = line.rstrip("\n")
                    if not line:
                        continue
                    if line.count(" ") < 2:
                        parts = line.split()
                        if len(parts) == 2:
                            key, value = parts
                        else:
                            continue
                    else:
                        split_pos = line.find(" ")
                        key = line[:split_pos]
                        value = line[split_pos + 1 :]
                    if key and value:
                        self.configuration[key] = value

    def write_configuration_to_file(self):
        with self._config_lock:
            with self.config_path.open("w") as out:
                for k, v in self.configuration.items():
                    out.write(f"{k} {v}\n")

    # --- Websocket handling ---
    async def handle_message(self, websocket: WebSocketServerProtocol, message: str):
        try:
            doc = json.loads(message)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON")
            return
        cmd = doc.get("command")
        if cmd == "getConfiguration":
            await self.send_configuration(websocket)
        elif cmd == "saveConfiguration":
            self.save_configuration(doc)
        elif cmd == "zeroImu":
            req = ImuOffsetService.Request()
            res = self.handle_zero_imu(req, ImuOffsetService.Response())
            await websocket.send(json.dumps({"zeroImu": res.value}))
        else:
            self.get_logger().error("Unknown command")

    async def send_configuration(self, ws: WebSocketServerProtocol):
        with self._config_lock:
            conf_array = [
                {"key": trim_spaces(k), "value": trim_spaces(v)}
                for k, v in self.configuration.items()
            ]
        payload = json.dumps({"configuration": conf_array})
        await ws.send(payload)

    def save_configuration(self, document: dict):
        if "configuration" not in document or not isinstance(document["configuration"], list):
            self.get_logger().error("Configuration array not found")
            return
        updated = False
        with self._config_lock:
            for entry in document["configuration"]:
                if isinstance(entry, dict) and "key" in entry and "value" in entry:
                    key = entry["key"]
                    value = entry["value"]
                    if key in self.configuration:
                        self.configuration[trim_spaces(key)] = trim_spaces(value)
                        updated = True
                    else:
                        self.get_logger().info("Key not found")
                else:
                    self.get_logger().info("Configuration object without key/value")
        if updated:
            self.write_configuration_to_file()
            self.broadcast_configuration()
            self.broadcast_imu_transform()
            self.update_hotspot_ssid()
        else:
            self.get_logger().error("Configuration array not found")

    # --- ROS interfaces ---
    def broadcast_configuration(self):
        with self._config_lock:
            for k, v in self.configuration.items():
                msg = Setting()
                msg.key = k
                msg.value = v
                self.config_pub.publish(msg)

    def broadcast_imu_transform(self):
        with self._config_lock:
            if not all(
                key in self.configuration
                for key in ["headingOffset", "pitchOffset", "rollOffset"]
            ):
                return
            try:
                heading = float(self.configuration["headingOffset"])
                pitch = float(self.configuration["pitchOffset"])
                roll = float(self.configuration["rollOffset"])
            except ValueError:
                return

        q = tf_transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(heading)
        )
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu"
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def handle_get_configuration(self, request: ConfigurationService.Request, response: ConfigurationService.Response):
        with self._config_lock:
            response.value = self.configuration.get(request.key, "")
        return response

    def handle_zero_imu(self, request: ImuOffsetService.Request, response: ImuOffsetService.Response):
        if not self.get_state_client.wait_for_service(timeout_sec=2.0):
            response.value = "get_state unavailable"
            return response
        future = self.get_state_client.call_async(GetStateService.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result():
            response.value = "get_state failed"
            return response
        state = future.result().state
        q = (
            state.imu.orientation.x,
            state.imu.orientation.y,
            state.imu.orientation.z,
            state.imu.orientation.w,
        )
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        with self._config_lock:
            self.configuration["headingOffset"] = str(-math.degrees(yaw))
            self.configuration["pitchOffset"] = str(-math.degrees(pitch))
            self.configuration["rollOffset"] = str(-math.degrees(roll))
            self.write_configuration_to_file()
        self.broadcast_configuration()
        self.broadcast_imu_transform()
        response.value = "ok"
        return response

    # --- Hotspot SSID update ---
    def update_hotspot_ssid(self):
        with self._config_lock:
            target_ssid = self.configuration.get("hotspotSSID")
        if not target_ssid:
            return
        try:
            if not self._is_same_ssid(target_ssid):
                subprocess.call(["nmcli", "con", "modify", "Hotspot", "802-11-wireless.ssid", target_ssid])
                subprocess.call(["nmcli", "con", "down", "Hotspot"])
                subprocess.call(["nmcli", "con", "up", "Hotspot"])
                if not self._is_same_ssid(target_ssid):
                    self.get_logger().error("Error in updating hotspotSSID")
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Failed to update hotspot SSID: {exc}")

    def _is_same_ssid(self, target: str) -> bool:
        try:
            result = subprocess.check_output(
                ["nmcli", "con", "show", "Hotspot"], text=True
            )
            current = ""
            for line in result.splitlines():
                if "wireless.ssid" in line:
                    # nmcli output format: "802-11-wireless.ssid:  <ssid>"
                    parts = line.split(":")
                    if len(parts) >= 2:
                        current = parts[1].strip()
                    break
            # Filter to alnum to mimic ROS1 clean-up
            current = "".join(ch for ch in current if ch.isalnum())
            target_clean = "".join(ch for ch in target if ch.isalnum())
            return current == target_clean
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ConfigWsNode()
    node.spin()


if __name__ == "__main__":
    main()
