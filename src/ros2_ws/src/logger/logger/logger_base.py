import abc
import base64
import collections
import json
import os
import ssl
import threading
import http.client
from typing import Optional, Tuple
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from setting_msg.msg import Setting
from setting_msg.srv import ConfigurationService
from logger_interfaces.srv import (
    GetLoggingStatus,
    ToggleLogging,
    GetLoggingMode,
    SetLoggingMode,
    TriggerTransfer,
)
from i2c_controller_service.srv import I2cControllerService
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo
from binary_stream_msg.msg import Stream as BinaryStream


class Geofence:
    """Lightweight WKT multipolygon parser/point-in-polygon, mirroring ROS1 utility."""

    def __init__(self, wkt: str = "MULTIPOLYGON(((180 90, -180 90, -180 -90, 180 -90, 180 90)))"):
        self.geofence = wkt
        self.polygons = self._parse_wkt(wkt)

    def contains(self, lon: float, lat: float) -> bool:
        return any(self._point_in_polygon(lon, lat, poly) for poly in self.polygons)

    @staticmethod
    def _point_in_polygon(lon: float, lat: float, polygon):
        n = len(polygon)
        inside = False
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            if ((lon > min(x1, x2)) and (lon <= max(x1, x2)) and (lat <= max(y1, y2))):
                if x2 != x1:
                    intersect = (lon - x1) * (y2 - y1) / (x2 - x1) + y1
                else:
                    intersect = y1
                if y1 == y2 or lat <= intersect:
                    inside = not inside
        return inside

    @staticmethod
    def _parse_wkt(wkt: str):
        if "(((" not in wkt or ")))" not in wkt:
            raise ValueError("Malformed WKT")
        inner = wkt.split("(((", 1)[1].rsplit(")))", 1)[0]
        polygons = []
        for poly_str in inner.split(")),(("):
            pts = []
            for pt in poly_str.split(","):
                parts = pt.strip().split()
                if len(parts) != 2:
                    continue
                lon = float(parts[0])
                lat = float(parts[1])
                pts.append((lon, lat))
            if len(pts) >= 3:
                polygons.append(pts)
        return polygons


class LoggerBase(Node, metaclass=abc.ABCMeta):
    """
    Base class for ROS2 logger nodes (text/binary).
    Mirrors the ROS1 service/topic interface and exposes hook methods
    for subclasses to implement file handling and transfers.
    """

    def __init__(self, node_name: str, output_folder: str, separator: str = ";") -> None:
        super().__init__(node_name)
        self.output_folder = output_folder
        self.separator = separator

        # State flags and config
        self.logging_mode = 1  # 1=AlwaysOn, 2=Manual, 3=SpeedBased
        self.speed_threshold_kmh = 5.0
        self.allow_toggle_without_gps = self.declare_parameter("allow_toggle_without_gps", False).value
        self.logger_enabled = False
        self.bootstrapped_gnss_time = False
        self.hdd_free_space_ok = True
        self.log_rotation_interval = self.declare_parameter("log_rotation_interval_seconds", 3600).value
        self.activated_transfer = False
        self.host = ""
        self.target = "/"
        self.port = "8080"
        self.api_key = ""
        self.geofence = Geofence()
        self.enable_geofence = False
        self.inside_geofence = False

        # File extensions for binary logs
        self.file_extension_sonar = ".son"
        self.file_extension_gps = ".gps"

        # Subscribers
        qos = 10
        self.gnss_sub = self.create_subscription(NavSatFix, "fix", self.gnss_callback, qos)
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, qos)
        self.depth_sub = self.create_subscription(PointStamped, "depth", self.sonar_callback, qos)
        self.speed_sub = self.create_subscription(Odometry, "speed", self.speed_callback, qos)
        self.lidar_sub = self.create_subscription(PointCloud2, "velodyne_points", self.lidar_callback, qos)
        self.vitals_sub = self.create_subscription(SysInfo, "vitals", self.vitals_callback, qos)
        self.config_sub = self.create_subscription(Setting, "configuration", self.configuration_callback, qos)
        self.gnss_stream_sub = self.create_subscription(BinaryStream, "gnss_bin_stream", self.gnss_stream_callback, qos)
        self.sonar_stream_sub = self.create_subscription(BinaryStream, "sonar_bin_stream", self.sonar_stream_callback, qos)

        # Services
        self.srv_get_status = self.create_service(GetLoggingStatus, "get_logging_status", self.get_logging_status)
        self.srv_toggle = self.create_service(ToggleLogging, "toggle_logging", self.toggle_logging)
        self.srv_get_mode = self.create_service(GetLoggingMode, "get_logging_mode", self.get_logging_mode_srv)
        self.srv_set_mode = self.create_service(SetLoggingMode, "set_logging_mode", self.set_logging_mode_srv)
        self.srv_trigger_transfer = self.create_service(
            TriggerTransfer, "trigger_transfer", self.handle_trigger_transfer
        )

        # Clients
        self.config_client = self.create_client(ConfigurationService, "get_configuration")
        self.i2c_client = self.create_client(I2cControllerService, "i2c_controller_service")

        self._lock = threading.Lock()
        self.last_rotation_time = self.get_clock().now()
        self.last_gnss_time = self.get_clock().now()
        self.speed_window = collections.deque(maxlen=120)

        # Timers
        self.gnss_watchdog = self.create_timer(1.0, self._gnss_watchdog_cb)

        # Initial configuration load (best effort)
        self._load_initial_config()
        self._set_led("led_nofix")

        self.get_logger().info(
            f"{node_name} ready (mode={self.logging_mode}, enabled={self.logger_enabled}, geofence={'on' if self.enable_geofence else 'off'})"
        )

    # --- Abstract hooks ---
    @abc.abstractmethod
    def init_logging(self):
        """Subclasses should initialize resources when logging starts."""

    @abc.abstractmethod
    def finalize_logging(self):
        """Subclasses should flush/close resources when logging stops."""

    @abc.abstractmethod
    def save_speed(self, msg: Odometry):
        pass

    @abc.abstractmethod
    def save_vitals(self, msg: SysInfo):
        pass

    @abc.abstractmethod
    def save_lidar(self, msg: PointCloud2):
        pass

    @abc.abstractmethod
    def save_depth(self, msg: PointStamped):
        pass

    @abc.abstractmethod
    def save_gnss(self, msg: NavSatFix):
        pass

    @abc.abstractmethod
    def save_imu(self, msg: Imu):
        pass

    @abc.abstractmethod
    def transfer(self) -> Tuple[bool, str]:
        """Perform file transfer; return (success: bool, message: str)."""

    # Shared transfer helper (base64 zip upload over HTTP/HTTPS)
    def _transfer_zips(self) -> Tuple[bool, str]:
        if not self.activated_transfer or not self.host:
            return False, "transfer disabled (apiServer not set)"

        target_dir = os.path.abspath(self.output_folder)
        if not os.path.isdir(target_dir):
            return False, f"output folder missing: {target_dir}"

        zips = sorted([f for f in os.listdir(target_dir) if f.endswith(".zip")])
        if not zips:
            return False, "no zip archives to transfer"

        for idx, zip_name in enumerate(zips, start=1):
            zip_path = os.path.join(target_dir, zip_name)
            try:
                payload = self._build_payload(zip_path)
                ok, msg = self._send_job(payload)
                if not ok:
                    return False, f"failed on {zip_name}: {msg}"
                os.remove(zip_path)
            except Exception as exc:  # noqa: BLE001
                return False, f"error on {zip_name}: {exc}"

        return True, f"transferred {len(zips)} archives"

    def _build_payload(self, zip_path: str) -> str:
        with open(zip_path, "rb") as f:
            data = f.read()
        b64 = base64.b64encode(data).decode("ascii")
        body = {"apiKey": self.api_key, "jobType": "Hydroball20", "fileData": b64}
        return json.dumps(body)

    def _send_job(self, body: str) -> Tuple[bool, str]:
        # Support host with or without scheme
        url = self.host
        if not url.startswith("http://") and not url.startswith("https://"):
            scheme = "https" if self.port == "443" else "http"
            url = f"{scheme}://{self.host}"
        parsed = urlparse(url)
        port = int(parsed.port) if parsed.port else int(self.port or 80)
        path = self.target if self.target.startswith("/") else f"/{self.target}"

        conn_cls = http.client.HTTPSConnection if parsed.scheme == "https" else http.client.HTTPConnection
        try:
            conn = conn_cls(parsed.hostname, port, context=ssl.create_default_context() if parsed.scheme == "https" else None, timeout=10)
            headers = {"Content-Type": "application/json", "Host": parsed.hostname}
            conn.request("POST", path, body=body, headers=headers)
            resp = conn.getresponse()
            if resp.status == 200:
                return True, "ok"
            return False, f"http {resp.status}"
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        finally:
            try:
                conn.close()
            except Exception:
                pass

    # Optional raw stream hooks
    def save_gnss_stream(self, stream: BinaryStream):
        return

    def save_sonar_stream(self, stream: BinaryStream):
        return

    # --- Helpers and configuration ---
    def _call_config(self, key: str, default: Optional[str] = None) -> Optional[str]:
        if not self.config_client.service_is_ready():
            try:
                self.config_client.wait_for_service(timeout_sec=2.0)
            except Exception:
                return default
        req = ConfigurationService.Request()
        req.key = key
        future = self.config_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result() is not None and future.result().value:
            return future.result().value
        return default

    def _load_initial_config(self):
        try:
            lm = self._call_config("loggingMode", None)
            if lm:
                self.logging_mode = int(lm)
        except Exception:
            pass
        try:
            spd = self._call_config("speedThresholdKmh", None)
            if spd:
                self.speed_threshold_kmh = float(spd)
        except Exception:
            pass
        geofence_wkt = self._call_config("geofence", None)
        if geofence_wkt:
            try:
                self.geofence = Geofence(geofence_wkt)
                self.enable_geofence = True
            except Exception as exc:
                self.get_logger().warn(f"Invalid geofence WKT: {exc}")
        api_server = self._call_config("apiServer", "")
        if api_server:
            self.host = api_server.strip()
            self.activated_transfer = True
        self.target = self._call_config("apiUrlPath", "/") or "/"
        self.port = self._call_config("apiPort", "8080") or "8080"
        self.api_key = (self._call_config("apiKey", "") or "").strip()
        self.log_rotation_interval = int(self._call_config("logRotationIntervalSeconds", str(self.log_rotation_interval)) or self.log_rotation_interval)
        fe_sonar = self._call_config("fileExtensionForSonarDatagram", ".son")
        fe_gps = self._call_config("fileExtensionForGpsDatagram", ".gps")
        if fe_sonar:
            self.file_extension_sonar = fe_sonar if fe_sonar.startswith(".") else "." + fe_sonar
        if fe_gps:
            self.file_extension_gps = fe_gps if fe_gps.startswith(".") else "." + fe_gps

    # --- LED helpers ---
    def _set_led(self, action: str):
        if not self.i2c_client.service_is_ready():
            try:
                self.i2c_client.wait_for_service(timeout_sec=0.5)
            except Exception:
                return
        req = I2cControllerService.Request()
        req.action2perform = action
        self.i2c_client.call_async(req)

    # --- Callbacks ---
    def get_logging_status(self, request, response):
        response.status = self.bootstrapped_gnss_time and self.logger_enabled
        return response

    def get_logging_mode_srv(self, request, response):
        response.logging_mode = int(self.logging_mode)
        return response

    def set_logging_mode_srv(self, request, response):
        mode = int(request.logging_mode)
        if mode in (1, 2, 3):
            self.logging_mode = mode
        else:
            self.get_logger().warn(f"Ignoring invalid logging mode {mode}")
        return response

    def toggle_logging(self, request, response):
        if self.allow_toggle_without_gps and not self.bootstrapped_gnss_time:
            self.bootstrapped_gnss_time = True

        if not self.bootstrapped_gnss_time:
            self.get_logger().warn("Cannot toggle logger because no gpsfix")
            response.logging_status = False
            return response

        with self._lock:
            if not self.logger_enabled and request.logging_enabled and self.hdd_free_space_ok:
                self.logger_enabled = True
                self.init_logging()
                self._set_led("led_recording")
            elif self.logger_enabled and not request.logging_enabled:
                self.logger_enabled = False
                self.finalize_logging()
                self._set_led("led_ready")

            response.logging_status = self.logger_enabled
            self.last_rotation_time = self.get_clock().now()
        return response

    def handle_trigger_transfer(self, request, response):
        try:
            ok, msg = self.transfer()
            response.success = bool(ok)
            response.message = msg
        except Exception as exc:  # pragma: no cover
            response.success = False
            response.message = f"Error: {exc}"
        return response

    def configuration_callback(self, setting: Setting):
        if setting.key == "loggingMode":
            try:
                mode = int(setting.value)
                self.logging_mode = mode
            except ValueError:
                self.get_logger().warn("Invalid loggingMode in configuration")
        elif setting.key == "speedThresholdKmh":
            try:
                self.speed_threshold_kmh = float(setting.value)
            except ValueError:
                pass
        elif setting.key == "geofence":
            try:
                self.geofence = Geofence(setting.value)
                self.enable_geofence = True
            except Exception as exc:
                self.get_logger().warn(f"Invalid geofence from config: {exc}")
        elif setting.key == "logRotationIntervalSeconds":
            try:
                self.log_rotation_interval = int(setting.value)
            except ValueError:
                pass
        elif setting.key == "apiServer":
            self.host = setting.value.strip()
            self.activated_transfer = bool(self.host)
        elif setting.key == "apiUrlPath":
            self.target = setting.value.strip() or "/"
        elif setting.key == "apiPort":
            self.port = setting.value.strip() or "8080"
        elif setting.key == "apiKey":
            self.api_key = setting.value.strip()

    def speed_callback(self, msg: Odometry):
        current_speed = msg.twist.twist.linear.y
        self.speed_window.append(current_speed)
        # Speed-based auto toggle
        if len(self.speed_window) == self.speed_window.maxlen and self.logging_mode == 3:
            avg_speed = sum(self.speed_window) / float(len(self.speed_window))
            if avg_speed > self.speed_threshold_kmh and not self.logger_enabled and self.hdd_free_space_ok:
                req = ToggleLogging.Request()
                req.logging_enabled = True
                _ = self.toggle_logging(req, ToggleLogging.Response())
            elif avg_speed < self.speed_threshold_kmh and self.logger_enabled:
                req = ToggleLogging.Request()
                req.logging_enabled = False
                _ = self.toggle_logging(req, ToggleLogging.Response())

        if self.logger_enabled:
            self.save_speed(msg)

    def vitals_callback(self, msg: SysInfo):
        # HDD safeguard
        if msg.freehdd < 1.0 and self.logger_enabled:
            req = ToggleLogging.Request()
            req.logging_enabled = False
            _ = self.toggle_logging(req, ToggleLogging.Response())
            self.hdd_free_space_ok = False
        elif msg.freehdd > 2.0:
            self.hdd_free_space_ok = True

        if self.logger_enabled:
            self.save_vitals(msg)

    def lidar_callback(self, msg: PointCloud2):
        if self.logger_enabled:
            self.save_lidar(msg)

    def sonar_callback(self, msg: PointStamped):
        if self.logger_enabled:
            self.save_depth(msg)

    def gnss_stream_callback(self, stream: BinaryStream):
        if self.logger_enabled:
            self.save_gnss_stream(stream)

    def sonar_stream_callback(self, stream: BinaryStream):
        if self.logger_enabled:
            self.save_sonar_stream(stream)

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.bootstrapped_gnss_time = True
            if self.enable_geofence:
                self.inside_geofence = self.geofence.contains(msg.longitude, msg.latitude)
        self.last_gnss_time = self.get_clock().now()

        # LED update on fix changes
        if msg.status.status < 0:
            self._set_led("led_nofix")
        else:
            self._set_led("gps_fix_recording" if self.logger_enabled else "gps_fix_ready")

        if self.logger_enabled:
            # Skip logging if geofence is enabled and outside
            if self.enable_geofence and not self.inside_geofence:
                return
            self.save_gnss(msg)

    def imu_callback(self, msg: Imu):
        if self.logger_enabled:
            self.save_imu(msg)

        # Handle rotation check on IMU as a heartbeat
        now = self.get_clock().now()
        if self.logger_enabled and (now - self.last_rotation_time).nanoseconds > self.log_rotation_interval * 1e9:
            self.finalize_logging()
            self.init_logging()
            self.last_rotation_time = now

    def _gnss_watchdog_cb(self):
        # If we have not heard GNSS for >1s, treat as no-fix for LED
        now = self.get_clock().now()
        if (now - self.last_gnss_time).nanoseconds > 1e9:
            self._set_led("led_nofix")


def spin_logger(node_cls, *args, **kwargs):
    rclpy.init()
    node: LoggerBase = node_cls(*args, **kwargs)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
