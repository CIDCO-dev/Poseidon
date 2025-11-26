import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from binary_stream_msg.msg import Stream
from setting_msg.msg import Setting
from setting_msg.srv import ConfigurationService

try:
    import serial
except ImportError:
    serial = None


class Imagenex852Node(Node):
    """
    ROS2 port of the Imagenex 852 sonar node (C variant).
    Publishes depth (+ ENU) and sonar_bin_stream, supports auto/manual trigger and config updates.
    """

    def __init__(self):
        super().__init__("sonar_imagenex852")
        self.device_path = self.declare_parameter("device_path", "/dev/sonar").value
        self.trigger_mode = self.declare_parameter("trigger_mode", "auto").value
        self.manual_ping_rate = float(self.declare_parameter("manual_ping_rate", 2.0).value)
        self.data_points = int(self.declare_parameter("data_points", 0).value)
        self.delay_nanoseconds = int(self.declare_parameter("delay_nanoseconds", 0).value)

        self.sonar_start_gain = 0x06
        self.sonar_range = 5
        self.sonar_absorption = 0x14
        self.sonar_pulse_length = 150

        self.pub_depth = self.create_publisher(PointStamped, "depth", 10)
        self.pub_depth_enu = self.create_publisher(PointStamped, "depth_enu", 10)
        self.pub_bin_stream = self.create_publisher(Stream, "sonar_bin_stream", 10)

        self.config_client = self.create_client(ConfigurationService, "get_configuration")
        self.config_sub = self.create_subscription(Setting, "configuration", self.configuration_callback, 10)

        self._mtx = threading.Lock()
        self._config_changed = False
        self._stop = threading.Event()
        self._ser: Optional["serial.Serial"] = None
        self._last_ping = time.time()

        self._load_configuration()

        if serial:
            try:
                self._ser = serial.Serial(
                    self.device_path,
                    baudrate=115200,
                    timeout=0.25,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                self.thread = threading.Thread(target=self.read_loop, daemon=True)
                self.thread.start()
                self.get_logger().info(f"Opened sonar on {self.device_path}")
            except Exception as exc:
                self.get_logger().error(f"Failed to open {self.device_path}: {exc}")
        else:
            self.get_logger().warn("pyserial not available; sonar_imagenex852 running idle")

    # --- Configuration ---
    def _load_configuration(self):
        if not self.config_client.wait_for_service(timeout_sec=0.5):
            return
        for key in ("sonarStartGain", "sonarRange", "sonarAbsorbtion", "sonarPulseLength"):
            req = ConfigurationService.Request()
            req.key = key
            future = self.config_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done() and future.result():
                val = future.result().value
                try:
                    ival = int(val)
                    with self._mtx:
                        if key == "sonarStartGain":
                            self.sonar_start_gain = ival
                        elif key == "sonarRange":
                            self.sonar_range = ival
                        elif key == "sonarAbsorbtion":
                            self.sonar_absorption = ival
                        elif key == "sonarPulseLength":
                            self.sonar_pulse_length = ival
                except Exception:
                    pass

    def configuration_callback(self, setting: Setting):
        with self._mtx:
            if setting.key == "sonarStartGain":
                self.sonar_start_gain = self._to_uint(setting.value, self.sonar_start_gain)
            elif setting.key == "sonarRange":
                self.sonar_range = self._to_uint(setting.value, self.sonar_range)
            elif setting.key == "sonarAbsorbtion":
                self.sonar_absorption = self._to_uint(setting.value, self.sonar_absorption)
            elif setting.key == "sonarPulseLength":
                self.sonar_pulse_length = self._to_uint(setting.value, self.sonar_pulse_length)
            else:
                return
            self._config_changed = True

    @staticmethod
    def _to_uint(val: str, default: int) -> int:
        try:
            return int(val)
        except Exception:
            return default

    # --- Serial handling ---
    def send_command(self):
        with self._mtx:
            range_val = self.sonar_range
            start_gain = self.sonar_start_gain
            absorp = self.sonar_absorption
            pulse = self.sonar_pulse_length
        trig_auto = self.trigger_mode != "manual"
        cmd = bytearray(27)
        cmd[0:2] = b"\xFE\x44"
        cmd[2] = 0x11  # headId
        cmd[3] = range_val & 0xFF
        cmd[6] = 0x43  # master+tx+send
        cmd[9] = start_gain & 0xFF
        cmd[11] = absorp & 0xFF
        cmd[15] = pulse & 0xFF
        cmd[17] = 0  # profileMinimumRange
        cmd[18] = 0  # reserved5[0]
        cmd[19] = 0  # reserved5[1]
        cmd[20] = 0x07 if trig_auto else 0x03
        cmd[21] = self.data_points & 0xFF
        cmd[23] = 0x00 if self.data_points > 0 else 0x01  # profile flag
        cmd[24] = 0x00  # switchDelay
        cmd[25] = 0x00  # frequency
        cmd[26] = 0xFD  # termination
        try:
            self._ser.write(cmd)
            self._ser.flush()
        except Exception as exc:
            self.get_logger().warn(f"Failed to send command: {exc}")

    def read_loop(self):
        # initial command
        self.send_command()
        self._last_ping = time.time()
        while not self._stop.is_set() and self._ser:
            try:
                # manual trigger resend
                if self.trigger_mode == "manual":
                    now = time.time()
                    if (now - self._last_ping) >= (1.0 / max(self.manual_ping_rate, 0.1)):
                        self.send_command()
                        self._last_ping = now
                if self._config_changed:
                    self.send_command()
                    with self._mtx:
                        self._config_changed = False

                b = self._ser.read(1)
                if b != b"I":
                    continue
                second = self._ser.read(1)
                if second not in (b"M", b"G", b"P"):
                    continue
                third = self._ser.read(1)
                if third != b"X":
                    continue
                hdr_rest = self._ser.read(9)
                if len(hdr_rest) < 9:
                    continue
                hdr = b"I" + second + b"X" + hdr_rest  # 12 bytes total
                self.process_data(hdr)
            except Exception as exc:
                self.get_logger().warn(f"Serial read error: {exc}")
                time.sleep(0.1)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def process_data(self, hdr_bytes: bytes):
        # hdr_bytes is 12 bytes
        if len(hdr_bytes) != 12:
            return
        magic = hdr_bytes[0:3]
        packet_type = magic[1:2]
        range_val = hdr_bytes[6]
        profile_range = hdr_bytes[7:9]
        serial_status = hdr_bytes[3]
        data_size = 0
        if packet_type == b"M":
            data_size = 252
        elif packet_type == b"G":
            data_size = 500
        elif packet_type == b"P":
            data_size = 0
        else:
            self.get_logger().error(f"Unknown packet type: {packet_type}")
            return

        # config change detection
        with self._mtx:
            if range_val != self.sonar_range:
                self._config_changed = True

        # warnings
        if not (serial_status & 0x01):
            self.get_logger().error("Echosounder not detected")
        if not (serial_status & 0x04):
            self.get_logger().error("Automatic trigger mode not supported. Pings may be unsynchronized")
        if serial_status & 0x80:
            self.get_logger().error("Character overrun detected")

        echo_data = b""
        if data_size > 0:
            echo_data = self._ser.read(data_size)
            if len(echo_data) != data_size:
                self.get_logger().error(f"Incomplete datapoints ({len(echo_data)}/{data_size})")
                return
        # consume termination 0xFC
        term = b""
        while term != b"\xFC":
            term = self._ser.read(1)
            if term == b"":
                break

        # compute depth
        profile_high = (profile_range[1] & 0x7E) >> 1
        profile_low = ((profile_range[1] & 0x01) << 7) | (profile_range[0] & 0x7F)
        depth_cm = (profile_high << 8) | profile_low
        depth_m = depth_cm / 100.0

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp.nanosec = self.delay_nanoseconds
        msg.header.frame_id = "sonar"
        msg.point.z = depth_m
        self.pub_depth.publish(msg)

        msg_enu = PointStamped()
        msg_enu.header = msg.header
        msg_enu.header.frame_id = "sonar_enu"
        msg_enu.point.z = -depth_m
        self.pub_depth_enu.publish(msg_enu)

        if data_size > 0:
            bs = Stream()
            bs.vector_length = len(hdr_bytes) + len(echo_data) + 1
            bs.stream = list(hdr_bytes + echo_data + b"\xFC")
            bs.time_stamp = int(time.time() * 1_000_000)
            bs.protocol = "IMX" if packet_type == b"M" else ("IGX" if packet_type == b"G" else "IPX")
            self.pub_bin_stream.publish(bs)

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Imagenex852Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
