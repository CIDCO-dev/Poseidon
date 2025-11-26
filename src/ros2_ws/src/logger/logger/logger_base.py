import abc
import threading
from typing import Optional

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
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo


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

        # State flags
        self.logging_mode = self.declare_parameter("logging_mode", 1).value  # 1=AlwaysOn
        self.allow_toggle_without_gps = self.declare_parameter(
            "allow_toggle_without_gps", False
        ).value
        self.logger_enabled = False
        self.bootstrapped_gnss_time = False
        self.hdd_free_space_ok = True
        self.log_rotation_interval = self.declare_parameter("log_rotation_interval_seconds", 3600).value

        # Subscribers
        qos = 10
        self.gnss_sub = self.create_subscription(NavSatFix, "fix", self.gnss_callback, qos)
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, qos)
        self.depth_sub = self.create_subscription(PointStamped, "depth", self.sonar_callback, qos)
        self.speed_sub = self.create_subscription(Odometry, "speed", self.speed_callback, qos)
        self.lidar_sub = self.create_subscription(
            PointCloud2, "velodyne_points", self.lidar_callback, qos
        )
        self.vitals_sub = self.create_subscription(SysInfo, "vitals", self.vitals_callback, qos)
        self.config_sub = self.create_subscription(
            Setting, "configuration", self.configuration_callback, qos
        )

        # Services
        self.srv_get_status = self.create_service(GetLoggingStatus, "get_logging_status", self.get_logging_status)
        self.srv_toggle = self.create_service(ToggleLogging, "toggle_logging", self.toggle_logging)
        self.srv_get_mode = self.create_service(GetLoggingMode, "get_logging_mode", self.get_logging_mode_srv)
        self.srv_set_mode = self.create_service(SetLoggingMode, "set_logging_mode", self.set_logging_mode_srv)
        self.srv_trigger_transfer = self.create_service(
            TriggerTransfer, "trigger_transfer", self.handle_trigger_transfer
        )

        # Config service client
        self.config_client = self.create_client(ConfigurationService, "get_configuration")

        self._lock = threading.Lock()
        self.last_rotation_time = self.get_clock().now()

        self.get_logger().info(
            f"{node_name} ready (mode={self.logging_mode}, enabled={self.logger_enabled})"
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
    def transfer(self):
        """Perform file transfer; return (success: bool, message: str)."""

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
            elif self.logger_enabled and not request.logging_enabled:
                self.logger_enabled = False
                self.finalize_logging()

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
        # Hook for subclasses if needed
        if setting.key == "loggingMode":
            try:
                mode = int(setting.value)
                self.logging_mode = mode
            except ValueError:
                self.get_logger().warn("Invalid loggingMode in configuration")

    def speed_callback(self, msg: Odometry):
        if self.logger_enabled:
            self.save_speed(msg)

    def vitals_callback(self, msg: SysInfo):
        if self.logger_enabled:
            self.save_vitals(msg)

    def lidar_callback(self, msg: PointCloud2):
        if self.logger_enabled:
            self.save_lidar(msg)

    def sonar_callback(self, msg: PointStamped):
        if self.logger_enabled:
            self.save_depth(msg)

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.bootstrapped_gnss_time = True
        if self.logger_enabled:
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


def spin_logger(node_cls, *args, **kwargs):
    rclpy.init()
    node: LoggerBase = node_cls(*args, **kwargs)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
