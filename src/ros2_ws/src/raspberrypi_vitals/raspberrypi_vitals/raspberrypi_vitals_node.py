import math
import subprocess
from pathlib import Path
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from raspberrypi_vitals_msg.msg import sysinfo as SysInfo
from i2c_controller_service.srv import I2cControllerService


class RpiVitalsNode(Node):
    def __init__(self) -> None:
        super().__init__('hbv')
        self.pub = self.create_publisher(SysInfo, 'vitals', QoSProfile(depth=10))
        self.i2c_client = self.create_client(I2cControllerService, 'i2c_controller_service')
        self.timer = self.create_timer(1.0, self.tick)
        self.sequence = 0
        self.debug = self.declare_parameter('debug_mode', False).value
        self.debug_warn = self.declare_parameter('debug_mode_warning_critical', False).value

    # Helpers to read system
    def read_float(self, path: Path) -> float:
        try:
            content = path.read_text().strip()
            return float(content)
        except Exception:
            if self.debug:
                self.get_logger().warn(f"Failed to read {path}")
            return float('nan')

    def read_loadavg(self) -> float:
        try:
            content = Path('/proc/loadavg').read_text().split()[0]
            return (float(content) / 8.0) * 100.0
        except Exception:
            if self.debug:
                self.get_logger().warn("Failed to read /proc/loadavg")
            return float('nan')

    def read_meminfo(self, key: str) -> float:
        try:
            with Path('/proc/meminfo').open() as f:
                for line in f:
                    if line.startswith(key):
                        parts = line.split()
                        return float(parts[1]) * 1024.0
        except Exception:
            if self.debug:
                self.get_logger().warn("Failed to read /proc/meminfo")
        return 0.0

    def get_free_ram_percent(self) -> float:
        total = self.read_meminfo('MemTotal:')
        free = self.read_meminfo('MemFree:')
        return (free / total * 100.0) if total else 0.0

    def get_free_hdd_percent(self) -> float:
        try:
            out = subprocess.check_output(['df', '--output=pcent', '/']).decode().strip().split('\n')[-1]
            used = float(out.strip().rstrip('%'))
            return 100.0 - used
        except Exception:
            if self.debug:
                self.get_logger().warn("Failed to run df")
            return float('nan')

    def call_i2c(self, action: str) -> float:
        if not self.i2c_client.wait_for_service(timeout_sec=0.5):
            if self.debug:
                self.get_logger().warn("i2c_controller_service unavailable")
            return float('nan')
        req = I2cControllerService.Request()
        req.action2perform = action
        future = self.i2c_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done() and future.result():
            return future.result().value
        return float('nan')

    def tick(self):
        msg = SysInfo()
        msg.header.seq = self.sequence = self.sequence + 1
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.cputemp = self.read_float(Path('/sys/class/thermal/thermal_zone0/temp')) / 1000.0
        msg.cpuload = self.read_loadavg()
        msg.freeram = self.get_free_ram_percent()
        msg.freehdd = self.get_free_hdd_percent()
        msg.uptime = self.read_float(Path('/proc/uptime'))

        msg.humidity = self._with_default(self.call_i2c('get_humidity'))
        msg.temperature = self._with_default(self.call_i2c('get_temperature'))
        msg.voltage = self._with_default(self.call_i2c('get_voltage'))
        msg.ledstate = self._with_default(self.call_i2c('get_led_state'), default=0.0)

        critical, crit_msg = self.is_critical(msg)
        warning, warn_msg = self.is_warning(msg)

        if critical:
            self.call_i2c('led_error')
            msg.status = crit_msg
        elif warning:
            self.call_i2c('led_warning')
            msg.status = warn_msg
        else:
            msg.status = 'Normal'
            # Let logger/i2c decide steady LED based on GPS/logging state

        self.pub.publish(msg)

    def is_critical(self, msg: SysInfo) -> Tuple[bool, str]:
        if 0.0 < msg.freehdd < 5.0:
            return True, f"[CRITICAL] Hard Disk Full detected (free HDD: {msg.freehdd:.2f}%)"
        if msg.voltage > 0 and msg.voltage <= 11.7:
            return True, f"[CRITICAL] Battery Under Voltage detected (voltage: {msg.voltage:.2f}V)"
        if msg.voltage >= 15.4:
            return True, f"[CRITICAL] Battery Overvoltage detected (voltage: {msg.voltage:.2f}V)"
        if msg.cputemp >= 80.0:
            return True, f"[CRITICAL] CPU Over Temperature detected (CPU Temp: {msg.cputemp:.2f}°C)"
        return False, "Normal"

    def is_warning(self, msg: SysInfo) -> Tuple[bool, str]:
        if 13.5 < msg.voltage <= 15.4:
            return True, f"[INFO] Battery is charging... (voltage: {msg.voltage:.2f}V)"
        if 0.0 < msg.voltage <= 11.9:
            return True, f"[WARNING] Battery Low Voltage detected (voltage: {msg.voltage:.2f}V)"
        if 0.0 < msg.freehdd < 20.0:
            return True, f"[WARNING] Low Hard Disk Space detected (free HDD: {msg.freehdd:.2f}%)"
        if msg.cputemp >= 75.0:
            return True, f"[WARNING] High CPU Temperature detected (CPU Temp: {msg.cputemp:.2f}°C)"
        return False, "Normal"

    @staticmethod
    def _with_default(value: float, default: float = -555.0) -> float:
        return value if not math.isnan(value) else default


def main(args=None):
    rclpy.init(args=args)
    node = RpiVitalsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
