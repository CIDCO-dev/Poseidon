import os
import time
import fcntl
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from i2c_controller_service.srv import I2cControllerService
from logger_interfaces.srv import GetLoggingStatus


I2C_SLAVE = 0x0703

# LED states matching the original enum in the ROS1 code
STATE_OFF = 0
STATE_READY = 1
STATE_RECORDING = 2
STATE_WARNING = 3
STATE_NOFIX = 4
STATE_ERROR = 5

STATE_MAP = {
    'off': STATE_OFF,
    'ready': STATE_READY,
    'recording': STATE_RECORDING,
    'warning': STATE_WARNING,
    'nofix': STATE_NOFIX,
    'error': STATE_ERROR,
}


class HIH8130Sensor:
    """Minimal HIH8130 driver mirroring the ROS1 implementation."""

    def __init__(self, logger: rclpy.node.Node, bus: str = '/dev/i2c-4', address: int = 0x27):
        self._logger = logger.get_logger()
        self._bus = bus
        self._address = address
        self._fd: Optional[int] = None
        self._open_bus()

    def _open_bus(self):
        try:
            self._fd = os.open(self._bus, os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_SLAVE, self._address)
        except OSError as exc:
            self._logger.warning(f'HIH8130 unavailable on {self._bus} (addr 0x{self._address:x}): {exc}')
            self._fd = None

    def _ensure_fd(self) -> bool:
        if self._fd is None:
            self._open_bus()
        return self._fd is not None

    def read_humidity(self) -> float:
        if not self._ensure_fd():
            raise RuntimeError('HIH8130 not available')
        # trigger a measurement by writing the address byte then read four bytes
        os.write(self._fd, bytes([self._address]))
        time.sleep(0.1)  # 100 ms
        data = os.read(self._fd, 4)
        if len(data) != 4:
            raise RuntimeError('HIH8130 read failed')
        humidity_raw = ((data[0] & 0x3F) << 8) + data[1]
        return (humidity_raw / float((1 << 14) - 2)) * 100.0

    def read_temperature(self) -> float:
        if not self._ensure_fd():
            raise RuntimeError('HIH8130 not available')
        os.write(self._fd, bytes([self._address]))
        time.sleep(0.1)
        data = os.read(self._fd, 4)
        if len(data) != 4:
            raise RuntimeError('HIH8130 read failed')
        temp_raw = (data[2] << 6) + (data[3] >> 2)
        return ((temp_raw / float((1 << 14) - 2)) * 165.0) - 40.0

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None


class INA238Sensor:
    """Minimal INA238 driver mirroring the ROS1 implementation."""

    def __init__(self, logger: rclpy.node.Node, bus: str = '/dev/i2c-4', address: int = 0x40):
        self._logger = logger.get_logger()
        self._bus = bus
        self._address = address
        self._fd: Optional[int] = None
        self._open_bus()

    def _open_bus(self):
        try:
            self._fd = os.open(self._bus, os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_SLAVE, self._address)
        except OSError as exc:
            self._logger.warning(f'INA238 unavailable on {self._bus} (addr 0x{self._address:x}): {exc}')
            self._fd = None

    def _ensure_fd(self) -> bool:
        if self._fd is None:
            self._open_bus()
        return self._fd is not None

    def _read_register(self, reg: int) -> bytes:
        if not self._ensure_fd():
            raise RuntimeError('INA238 not available')
        os.write(self._fd, bytes([reg]))
        data = os.read(self._fd, 2)
        if len(data) != 2:
            raise RuntimeError('INA238 read failed')
        return data

    def read_voltage(self) -> float:
        data = self._read_register(0x05)
        # ((MSB * 256 + LSB) * 3.125 / 1000) + diode drop
        return ((data[0] * 256 + data[1]) * 3.125 / 1000.0) + 0.47

    def read_shunt_voltage(self) -> float:
        data = self._read_register(0x04)
        return (data[0] * 256 + data[1]) * 5.0 / 1000.0

    def read_temperature(self) -> float:
        data = self._read_register(0x06)
        raw_temperature = (data[0] * 256 + data[1]) >> 4
        if raw_temperature & 0x800:
            raw_temperature = -((~raw_temperature & 0xFFF) + 1)
        return (raw_temperature * 125.0) / 1000.0

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None


class LEDController:
    """Mirror of ROS1 LED_Ctrl: publish state on topic and remember last state."""

    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(String, 'led_control', 10)
        self._last_state = 'off'

    def set_led(self, mode: str) -> bool:
        self._last_state = mode
        msg = String()
        msg.data = mode
        self._pub.publish(msg)
        return True

    def get_state_value(self) -> float:
        return float(STATE_MAP.get(self._last_state, STATE_OFF))


class I2cControllerNode(Node):
    def __init__(self):
        super().__init__('i2c_controller')
        self.board_version = float(self.declare_parameter('board_version', 2.0).value)
        self._lock = threading.Lock()

        self.led_controller = LEDController(self)
        self.weather_sensor = HIH8130Sensor(self) if self.board_version > 2.0 else None
        self.power_sensor = INA238Sensor(self) if self.board_version > 2.0 else None

        self.logger_status = self.create_client(GetLoggingStatus, 'get_logging_status')
        self.logger_status.wait_for_service(timeout_sec=2.0)

        self.warning_timer = None
        self.error_timer = None

        self.srv = self.create_service(I2cControllerService, 'i2c_controller_service', self.handle_request)
        self.get_logger().info('i2c_controller node ready (board version %.2f)' % self.board_version)

    # --- LED helpers -----------------------------------------------------
    def _reset_warning_timer(self):
        if self.warning_timer:
            self.warning_timer.cancel()
        self.warning_timer = self.create_timer(5.0, self._warning_timer_callback)

    def _reset_error_timer(self):
        if self.error_timer:
            self.error_timer.cancel()
        self.error_timer = self.create_timer(10.0, self._error_timer_callback)

    def _warning_timer_callback(self):
        # Single-shot behaviour: cancel immediately
        if self.warning_timer:
            self.warning_timer.cancel()
            self.warning_timer = None
        state = self.led_controller.get_state_value()
        if state <= STATE_WARNING:
            self._set_logger_recording_status()

    def _error_timer_callback(self):
        if self.error_timer:
            self.error_timer.cancel()
            self.error_timer = None
        state = self.led_controller.get_state_value()
        if state not in (STATE_WARNING, STATE_NOFIX):
            self._set_logger_recording_status()

    def _call_get_logging_status(self) -> Optional[bool]:
        if not self.logger_status.service_is_ready():
            self.logger_status.wait_for_service(timeout_sec=0.5)
        if not self.logger_status.service_is_ready():
            self.get_logger().warn('get_logging_status service not available')
            return None
        req = GetLoggingStatus.Request()
        future = self.logger_status.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().warn('get_logging_status call timed out')
            return None
        result = future.result()
        if result is None:
            self.get_logger().warn('get_logging_status returned no result')
            return None
        return bool(result.status)

    def _set_logger_recording_status(self):
        status = self._call_get_logging_status()
        if status is None:
            return
        mode = 'recording' if status else 'ready'
        self.led_controller.set_led(mode)

    def _is_error_on(self) -> bool:
        return self.led_controller.get_state_value() == STATE_ERROR

    def _is_no_fix_on(self) -> bool:
        return self.led_controller.get_state_value() == STATE_NOFIX

    def _is_warning_on(self) -> bool:
        return self.led_controller.get_state_value() == STATE_WARNING

    # --- Service handling ------------------------------------------------
    def handle_request(self, request, response):
        action = request.action2perform
        with self._lock:
            try:
                if self.board_version <= 2.0:
                    response.value = -555.0
                    return response

                if action == 'get_led_state':
                    response.value = self.led_controller.get_state_value()

                elif action == 'led_error':
                    self._reset_error_timer()
                    self.led_controller.set_led('error')
                    response.value = self.led_controller.get_state_value()

                elif action == 'led_ready':
                    if not (self._is_error_on() or self._is_no_fix_on() or self._is_warning_on()):
                        self.led_controller.set_led('ready')
                    response.value = self.led_controller.get_state_value()

                elif action == 'led_recording':
                    if not (self._is_error_on() or self._is_no_fix_on() or self._is_warning_on()):
                        self.led_controller.set_led('recording')
                    response.value = self.led_controller.get_state_value()

                elif action == 'led_warning':
                    if not self._is_error_on() and not self._is_no_fix_on():
                        self._reset_warning_timer()
                        self.led_controller.set_led('warning')
                    response.value = self.led_controller.get_state_value()

                elif action == 'led_nofix':
                    if not self._is_error_on():
                        self.led_controller.set_led('nofix')
                    response.value = self.led_controller.get_state_value()

                elif action == 'gps_fix_ready':
                    if not self._is_error_on() and not self._is_warning_on():
                        self.led_controller.set_led('ready')
                    response.value = self.led_controller.get_state_value()

                elif action == 'gps_fix_recording':
                    if not self._is_error_on() and not self._is_warning_on():
                        self.led_controller.set_led('recording')
                    response.value = self.led_controller.get_state_value()

                elif action == 'get_humidity':
                    response.value = self.weather_sensor.read_humidity() if self.weather_sensor else -555.0

                elif action == 'get_temperature':
                    response.value = self.weather_sensor.read_temperature() if self.weather_sensor else -555.0

                elif action == 'get_voltage':
                    response.value = self.power_sensor.read_voltage() if self.power_sensor else -555.0

                elif action == 'get_shuntVoltage':
                    response.value = self.power_sensor.read_shunt_voltage() if self.power_sensor else -555.0

                elif action == 'get_batteryTemperature':
                    response.value = self.power_sensor.read_temperature() if self.power_sensor else -555.0

                else:
                    self.get_logger().error(f'i2c_controller invalid request: {action}')
                    response.value = -555.0

            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'i2c_controller failed on {action}: {exc}')
                response.value = -555.0

        return response

    # --- Cleanup ---------------------------------------------------------
    def destroy_node(self):
        if self.warning_timer:
            self.warning_timer.cancel()
        if self.error_timer:
            self.error_timer.cancel()
        if self.weather_sensor:
            self.weather_sensor.close()
        if self.power_sensor:
            self.power_sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = I2cControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
