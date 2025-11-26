import os
import zipfile
from pathlib import Path
from typing import Tuple

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo

from .logger_base import LoggerBase


class LoggerText(LoggerBase):
    """ROS2 port of the text logger: writes CSV-like text files per sensor, rotates, optional zip."""

    def __init__(self, output_folder: str, separator: str = ";") -> None:
        super().__init__("logger_text_node", output_folder, separator)
        self.tmp_dir = Path("/tmp")
        self.files = {}
        self.last_gnss_ts = 0
        self.last_imu_ts = 0
        self.last_sonar_ts = 0
        self.last_lidar_ts = 0
        self.last_speed_ts = 0
        self.last_vitals_ts = 0
        self.date_string = ""

    # LoggerBase hooks
    def init_logging(self):
        self.date_string = self._timestamp_prefix()
        self.files = {
            "gnss": self._open_file(self.date_string + "_gnss.txt", f"Timestamp{self.separator}Longitude{self.separator}Latitude{self.separator}EllipsoidalHeight{self.separator}Status{self.separator}Service\n"),
            "imu": self._open_file(self.date_string + "_imu.txt", f"Timestamp{self.separator}Heading{self.separator}Pitch{self.separator}Roll\n"),
            "sonar": self._open_file(self.date_string + "_sonar.txt", f"Timestamp{self.separator}Depth\n"),
            "lidar": self._open_file(self.date_string + "_lidar.txt", f"Timestamp{self.separator}Points\n"),
            "speed": self._open_file(self.date_string + "_speed.txt", f"Timestamp{self.separator}Speed\n"),
            "vitals": self._open_file(self.date_string + "_vitals.txt", f"Timestamp{self.separator}CPU Temp{self.separator}CPU Used{self.separator}FREE RAM{self.separator}FREE HDD{self.separator}Uptime{self.separator}Humidity{self.separator}System Temp{self.separator}Supply Voltage\n"),
        }

    def finalize_logging(self):
        target_dir = Path(self.output_folder)
        target_dir.mkdir(parents=True, exist_ok=True)
        for name, f in list(self.files.items()):
            try:
                fname = Path(f.name).name
                f.close()
                src = Path(f.name)
                dest = target_dir / fname
                try:
                    src.replace(dest)
                except Exception:
                    pass
            except Exception:
                pass
        self.files = {}
        # Zip logs
        if self.date_string:
            zip_name = target_dir / f"{self.date_string}.zip"
            with zipfile.ZipFile(zip_name, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for p in target_dir.iterdir():
                    if p.is_file() and p.name.startswith(self.date_string):
                        zf.write(p, arcname=p.name)

    def save_gnss(self, msg: NavSatFix):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts <= self.last_gnss_ts or "gnss" not in self.files:
            return
        f = self.files["gnss"]
        f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}{msg.longitude:.10f}{self.separator}{msg.latitude:.10f}{self.separator}{msg.altitude:.3f}{self.separator}{msg.status.status}{self.separator}{msg.status.service}\n")
        f.flush()
        self.last_gnss_ts = ts

    def save_imu(self, msg: Imu):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts <= self.last_imu_ts or "imu" not in self.files:
            return
        f = self.files["imu"]
        f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}{msg.orientation.x:.3f}{self.separator}{msg.orientation.y:.3f}{self.separator}{msg.orientation.z:.3f}\n")
        f.flush()
        self.last_imu_ts = ts

    def save_depth(self, msg: PointStamped):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts <= self.last_sonar_ts or "sonar" not in self.files:
            return
        f = self.files["sonar"]
        f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}{msg.point.z:.3f}\n")
        f.flush()
        self.last_sonar_ts = ts

    def save_lidar(self, msg: PointCloud2):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts <= self.last_lidar_ts or "lidar" not in self.files:
            return
        f = self.files["lidar"]
        f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}points\n")
        f.flush()
        self.last_lidar_ts = ts

    def save_speed(self, msg: Odometry):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts <= self.last_speed_ts or "speed" not in self.files:
            return
        f = self.files["speed"]
        f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}{msg.twist.twist.linear.y:.3f}\n")
        f.flush()
        self.last_speed_ts = ts

    def save_vitals(self, msg: SysInfo):
        ts = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1_000
        if ts - self.last_vitals_ts < 60 * 1_000_000 or "vitals" not in self.files:
            return
        f = self.files["vitals"]
        f.write(
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}{self.separator}"
            f"{msg.cputemp:.3f}{self.separator}{msg.cpuload:.3f}{self.separator}"
            f"{msg.freeram:.3f}{self.separator}{msg.freehdd:.3f}{self.separator}"
            f"{msg.uptime:.3f}{self.separator}{msg.humidity:.3f}{self.separator}"
            f"{msg.temperature:.3f}{self.separator}{msg.voltage:.3f}\n"
        )
        f.flush()
        self.last_vitals_ts = ts

    def transfer(self) -> Tuple[bool, str]:
        # Placeholder: remote transfer not implemented in this port
        return False, "transfer not implemented"

    # Helpers
    def _open_file(self, filename: str, header: str):
        path = self.tmp_dir / filename
        os.makedirs(self.tmp_dir, exist_ok=True)
        f = open(path, "a")
        f.write(header)
        return f

    def _timestamp_prefix(self) -> str:
        now = self.get_clock().now().to_msg()
        return f"{now.sec}"
