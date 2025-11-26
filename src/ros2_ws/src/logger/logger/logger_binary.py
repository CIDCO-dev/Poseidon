import os
from pathlib import Path
from typing import Tuple

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo

from .logger_base import LoggerBase


class LoggerBinary(LoggerBase):
    """
    ROS2 port of the binary logger (simplified): stores raw GNSS/sonar streams and basic data.
    """

    def __init__(self, output_folder: str) -> None:
        super().__init__("logger_binary_node", output_folder, separator=";")
        self.tmp_dir = Path("/tmp")
        self.files = {}

    def init_logging(self):
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
        prefix = self._timestamp_prefix()
        self.files = {
            "gnss_raw": open(self.tmp_dir / f"{prefix}.gnss", "wb"),
            "sonar_raw": open(self.tmp_dir / f"{prefix}.son", "wb"),
        }

    def finalize_logging(self):
        target_dir = Path(self.output_folder)
        target_dir.mkdir(parents=True, exist_ok=True)
        for f in self.files.values():
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

    def transfer(self) -> Tuple[bool, str]:
        return False, "transfer not implemented"

    def save_gnss(self, msg: NavSatFix):
        # Raw GNSS not available from NavSatFix; nothing to do.
        return

    def save_imu(self, msg: Imu):
        return

    def save_depth(self, msg: PointStamped):
        if "sonar_raw" in self.files:
            try:
                self.files["sonar_raw"].write(str(msg.point.z).encode() + b"\n")
                self.files["sonar_raw"].flush()
            except Exception:
                pass

    def save_lidar(self, msg: PointCloud2):
        return

    def save_speed(self, msg: Odometry):
        return

    def save_vitals(self, msg: SysInfo):
        return

    def _timestamp_prefix(self) -> str:
        now = self.get_clock().now().to_msg()
        return f"{now.sec}"
