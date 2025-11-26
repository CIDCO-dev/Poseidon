import math
import struct
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gnss_status_msg.msg import GnssDiagnostic
from binary_stream_msg.msg import Stream

try:
    import serial
except ImportError:
    serial = None

UBX_SYNC = b"\xb5\x62"


class GnssZedF9PNode(Node):
    """
    ROS2 port of the ZED-F9P driver:
    - reads UBX over serial, publishes gnss_bin_stream, speed (Odometry), and gnss_status
    - logs raw UBX frames to rotating files (hourly) if output_folder is set
    - fallback dummy publisher if serial is unavailable
    """

    def __init__(self):
        super().__init__('zedf9p')
        self.speed_pub = self.create_publisher(Odometry, 'speed', 10)
        self.status_pub = self.create_publisher(GnssDiagnostic, 'gnss_status', 10)
        self.stream_pub = self.create_publisher(Stream, 'gnss_bin_stream', 10)

        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.baudrate = self.declare_parameter('baudrate', 460800).value
        self.output_folder = Path(self.declare_parameter('output_folder', '/tmp').value)
        self.rotation_seconds = int(self.declare_parameter('log_rotation_seconds', 3600).value)
        self._stop = threading.Event()
        self._current_log: Optional[Path] = None
        self._current_file = None
        self._last_rotation = time.time()

        if serial:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(f"Opened serial port {self.serial_port} @ {self.baudrate}")
                self.thread = threading.Thread(target=self.read_loop, daemon=True)
                self.thread.start()
            except Exception as exc:
                self.get_logger().error(f"Failed to open {self.serial_port}: {exc}, running in dummy mode")
                self.ser = None
                self.timer = self.create_timer(1.0, self.publish_dummy)
        else:
            self.get_logger().warn("pyserial not available; running in dummy mode")
            self.ser = None
            self.timer = self.create_timer(1.0, self.publish_dummy)

    def destroy_node(self):
        self._stop.set()
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        if self._current_file:
            try:
                self._current_file.close()
            except Exception:
                pass
        super().destroy_node()

    def publish_dummy(self):
        ts = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = ts
        odom.twist.twist.linear.y = abs(math.sin(time.time())) * 5.0
        self.speed_pub.publish(odom)

        status = GnssDiagnostic()
        status.header.stamp = ts
        status.fix_type = 3
        status.diff_soln = True
        status.carr_soln = 2
        status.num_sv = 16
        status.horizontal_accuracy = 0.5
        status.vertical_accuracy = 0.8
        self.status_pub.publish(status)

    def read_loop(self):
        while not self._stop.is_set() and self.ser:
            try:
                if self.ser.read(2) != UBX_SYNC:
                    continue
                hdr = self.ser.read(4)
                if len(hdr) < 4:
                    continue
                msg_class, msg_id, length = struct.unpack('<BBH', hdr)
                payload = self.ser.read(length)
                cksum = self.ser.read(2)
                if len(payload) != length or len(cksum) != 2:
                    continue
                frame = UBX_SYNC + hdr + payload + cksum
                self._maybe_rotate_log()
                self._write_log(frame)
                self.publish_stream(frame)
                if msg_class == 0x01 and msg_id == 0x07:  # NAV-PVT
                    self.handle_nav_pvt(payload)
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"Serial read error: {exc}")
                time.sleep(0.1)

    def _maybe_rotate_log(self):
        if not self.output_folder:
            return
        now = time.time()
        if not self._current_file or (now - self._last_rotation) > self.rotation_seconds:
            if self._current_file:
                try:
                    self._current_file.close()
                except Exception:
                    pass
            self.output_folder.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._current_log = self.output_folder / f"{ts}.ubx"
            self._current_file = open(self._current_log, "ab")
            self._last_rotation = now

    def _write_log(self, frame: bytes):
        if self._current_file:
            try:
                self._current_file.write(frame)
                self._current_file.flush()
            except Exception:
                pass

    def publish_stream(self, frame: bytes):
        msg = Stream()
        msg.protocol = 'UBX'
        msg.time_stamp = int(time.time() * 1_000_000)
        msg.vector_length = len(frame)
        msg.stream = list(frame)
        self.stream_pub.publish(msg)

    def handle_nav_pvt(self, payload: bytes):
        if len(payload) < 92:
            return
        try:
            speed_mm_s = struct.unpack_from('<i', payload, 60)[0]
            speed_kmh = speed_mm_s * 3.6 / 1000.0
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.twist.twist.linear.y = speed_kmh
            self.speed_pub.publish(odom)

            fix_type = payload[20]
            flags = payload[21]
            num_sv = payload[23]
            h_acc_mm = struct.unpack_from('<I', payload, 32)[0]
            v_acc_mm = struct.unpack_from('<I', payload, 36)[0]

            status = GnssDiagnostic()
            status.header.stamp = odom.header.stamp
            status.fix_type = fix_type
            status.diff_soln = bool((flags >> 2) & 0x01)
            status.carr_soln = (flags >> 6) & 0x03
            status.num_sv = num_sv
            status.horizontal_accuracy = h_acc_mm / 1000.0
            status.vertical_accuracy = v_acc_mm / 1000.0
            self.status_pub.publish(status)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse NAV-PVT: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GnssZedF9PNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
