import zipfile
from pathlib import Path
from typing import Tuple

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo
from binary_stream_msg.msg import Stream as BinaryStream

from .logger_base import LoggerBase


class LoggerBinary(LoggerBase):
    """
    ROS2 port of the binary logger: stores raw GNSS and sonar datagrams plus basic depth.
    """

    def __init__(self, output_folder: str) -> None:
        super().__init__("logger_binary_node", output_folder, separator=";")
        self.tmp_dir = Path("/tmp")
        self.files = {}
        self.prefix = ""

    def init_logging(self):
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
        self.prefix = self._timestamp_prefix()
        self.files = {
            "gnss_raw": open(self.tmp_dir / f"{self.prefix}{self.file_extension_gps}", "ab"),
            "sonar_raw": open(self.tmp_dir / f"{self.prefix}{self.file_extension_sonar}", "ab"),
        }

    def finalize_logging(self):
        target_dir = Path(self.output_folder)
        target_dir.mkdir(parents=True, exist_ok=True)
        for f in list(self.files.values()):
            try:
                fname = Path(f.name).name
                f.close()
                Path(f.name).replace(target_dir / fname)
            except Exception:
                pass
        self.files = {}
        # Zip the session files
        if self.prefix:
            zip_path = target_dir / f"{self.prefix}.zip"
            with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for p in target_dir.iterdir():
                    if p.is_file() and p.name.startswith(self.prefix):
                        zf.write(p, arcname=p.name)

    def transfer(self) -> Tuple[bool, str]:
        return self._transfer_zips()

    def save_gnss(self, msg: NavSatFix):
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

    def save_gnss_stream(self, stream: BinaryStream):
        if "gnss_raw" not in self.files:
            return
        try:
            self.files["gnss_raw"].write(bytes(stream.stream))
            self.files["gnss_raw"].flush()
        except Exception:
            pass

    def save_sonar_stream(self, stream: BinaryStream):
        if "sonar_raw" not in self.files:
            return
        try:
            self.files["sonar_raw"].write(bytes(stream.stream))
            self.files["sonar_raw"].flush()
        except Exception:
            pass

    def _timestamp_prefix(self) -> str:
        now = self.get_clock().now().to_msg()
        return f"{now.sec}"
