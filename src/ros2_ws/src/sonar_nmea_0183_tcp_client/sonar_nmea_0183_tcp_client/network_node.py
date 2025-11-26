import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


def compute_checksum(data: str) -> int:
    checksum = 0
    for ch in data:
        checksum ^= ord(ch)
    return checksum


def parse_checksum(byte_str: str) -> Optional[int]:
    if len(byte_str) < 2:
        return None
    try:
        return int(byte_str[:2], 16)
    except ValueError:
        return None


def validate_checksum(sentence: str) -> bool:
    if not sentence.startswith("$"):
        return False
    if "*" not in sentence:
        return False
    star = sentence.find("*")
    bytes_str = sentence[1:star]
    ctrl = sentence[star + 1 :]
    control = parse_checksum(ctrl)
    if control is None:
        return False
    return compute_checksum(bytes_str) == control


class BaseNmeaClient(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.pub_depth = self.create_publisher(PointStamped, "depth", 10)
        self.pub_fix = self.create_publisher(NavSatFix, "fix", 10)
        self.pub_speed = self.create_publisher(Odometry, "speed", 10)
        self.depth_seq = 0
        self.gps_seq = 0
        self.speed_seq = 0

    def publish_depth(self, depth_m: float):
        msg = PointStamped()
        self.depth_seq += 1
        msg.header.seq = self.depth_seq
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.z = depth_m
        self.pub_depth.publish(msg)

    def publish_fix(self, lat: float, lon: float, alt: float, status: int):
        msg = NavSatFix()
        self.gps_seq += 1
        msg.header.seq = self.gps_seq
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.status.status = status
        msg.status.service = 1
        msg.position_covariance_type = 0
        self.pub_fix.publish(msg)

    def publish_speed(self, speed_kmh: float):
        msg = Odometry()
        self.speed_seq += 1
        msg.header.seq = self.speed_seq
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.y = speed_kmh
        self.pub_speed.publish(msg)

    # --- Parsers ---
    def parse_gga(self, line: str):
        # $G?GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        try:
            parts = line.split(",")
            if len(parts) < 10:
                return
            lat_raw = parts[2]
            lat_dir = parts[3]
            lon_raw = parts[4]
            lon_dir = parts[5]
            quality = int(parts[6]) if parts[6] else 0
            altitude = float(parts[9]) if parts[9] else 0.0

            lat_deg = int(float(lat_raw) / 100) if lat_raw else 0
            lat_min = (float(lat_raw) - lat_deg * 100) / 60.0 if lat_raw else 0.0
            lat = (lat_deg + lat_min) * (1 if lat_dir == "N" else -1)

            lon_deg = int(float(lon_raw) / 100) if lon_raw else 0
            lon_min = (float(lon_raw) - lon_deg * 100) / 60.0 if lon_raw else 0.0
            lon = (lon_deg + lon_min) * (1 if lon_dir == "E" else -1)

            status = -1
            if quality == 1:
                status = 0
            elif quality == 2:
                status = 2

            self.publish_fix(lat, lon, altitude, status)
        except Exception:
            return

    def parse_dbt(self, line: str):
        # $--DBT,xx.x,f,xx.x,M,xx.x,F*hh
        try:
            parts = line.split(",")
            if len(parts) < 5:
                return
            depth_m = float(parts[3]) if parts[3] else None
            if depth_m is not None:
                self.publish_depth(depth_m)
        except Exception:
            return

    def parse_dpt(self, line: str):
        # $--DPT,xx.x,xx.x,xx.x*hh
        try:
            parts = line.split(",")
            if len(parts) >= 2:
                depth_m = float(parts[1]) if parts[1] else None
                if depth_m is not None:
                    self.publish_depth(depth_m)
        except Exception:
            return

    def parse_ads(self, line: str):
        # $ISADS,ddd.ddd,M,tt.t,C*hh
        try:
            parts = line.split(",")
            if len(parts) >= 3:
                depth_m = float(parts[1]) if parts[1] else None
                if depth_m is not None:
                    self.publish_depth(depth_m)
        except Exception:
            return

    def parse_vtg(self, line: str):
        # $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
        try:
            parts = line.split(",")
            if len(parts) >= 8:
                speed_kmh = float(parts[7]) if parts[7] else None
                if speed_kmh is not None:
                    self.publish_speed(speed_kmh)
        except Exception:
            return


class NetworkNmeaNode(BaseNmeaClient):
    def __init__(self):
        super().__init__("nmea_network")
        self.ip = self.declare_parameter("ip_address", "127.0.0.1").value
        self.port = int(self.declare_parameter("port", 5000).value)
        self.use_depth = bool(self.declare_parameter("use_depth", True).value)
        self.use_position = bool(self.declare_parameter("use_position", True).value)
        self.use_speed = bool(self.declare_parameter("use_speed", True).value)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        retry_delay = 1.0
        while not self._stop.is_set():
            try:
                self.get_logger().info(f"Connecting to {self.ip}:{self.port}")
                with socket.create_connection((self.ip, self.port), timeout=5) as sock:
                    sock.settimeout(1.0)
                    buf = ""
                    while not self._stop.is_set():
                        try:
                            data = sock.recv(4096)
                            if not data:
                                break
                            buf += data.decode(errors="ignore")
                            while "\n" in buf:
                                line, buf = buf.split("\n", 1)
                                self.handle_line(line.strip())
                        except socket.timeout:
                            # allow reconnect
                            continue
                self.get_logger().warn("Disconnected, retrying...")
                time.sleep(retry_delay)
            except Exception as exc:
                self.get_logger().warn(f"Connection error: {exc}")
                time.sleep(retry_delay)

    def handle_line(self, line: str):
        if not line.startswith("$"):
            return
        if "*" in line and not validate_checksum(line):
            self.get_logger().warn("Checksum error")
            return

        if line.startswith("$") and len(line) > 6:
            sentence = line[3:6]
            if sentence == "GGA" and self.use_position:
                self.parse_gga(line)
            elif sentence == "DBT" and self.use_depth:
                self.parse_dbt(line)
            elif sentence == "DPT" and self.use_depth:
                self.parse_dpt(line)
            elif sentence == "ADS" and self.use_depth:
                self.parse_ads(line)
            elif sentence == "VTG" and self.use_speed:
                self.parse_vtg(line)

    def destroy_node(self):
        self._stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetworkNmeaNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
