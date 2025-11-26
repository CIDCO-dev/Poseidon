import math
import struct
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_srvs.srv import Trigger

try:
    import smbus2
except ImportError:
    smbus2 = None

# BNO055 registers/constants (subset)
BNO055_ADDRESS_A = 0x28
BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_SYS_TRIGGER_ADDR = 0x3F
BNO055_UNIT_SEL_ADDR = 0x3B
BNO055_PAGE_ID_ADDR = 0x07
BNO055_CALIB_STAT_ADDR = 0x35
BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20
BNO055_TEMP_ADDR = 0x34
BNO055_EULER_H_LSB_ADDR = 0x1A

MODE_CONFIG = 0x00
MODE_NDOF = 0x0C
POWER_NORMAL = 0x00


class Bno055Node(Node):
    """
    Minimal ROS2 port of the BNO055 I2C node:
    - Publishes /imu/data, /mag, /temperature
    - Exposes reset and calibrate services (Trigger)
    - Reads calibration status
    """

    def __init__(self):
        super().__init__("bno055_node")
        self.bus_num = int(self.declare_parameter("bus", 4).value)
        self.address = int(self.declare_parameter("address", BNO055_ADDRESS_A).value)
        self.frame_id = self.declare_parameter("frame_id", "imu").value
        self.calibration_file = Path(
            self.declare_parameter("calibration_file", "/opt/Poseidon/calibration.dat").value
        )
        self.rate_hz = int(self.declare_parameter("rate", 100).value)

        qos = QoSProfile(depth=10)
        self.pub_imu = self.create_publisher(Imu, "imu/data", qos)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", qos)
        self.pub_temp = self.create_publisher(Temperature, "imu/temp", qos)

        self.srv_reset = self.create_service(Trigger, "reset", self.handle_reset)
        self.srv_calibrate = self.create_service(Trigger, "calibrate", self.handle_calibrate)

        self.bus: Optional["smbus2.SMBus"] = None
        if smbus2:
            try:
                self.bus = smbus2.SMBus(self.bus_num)
                self._initialize_sensor()
                self.timer = self.create_timer(1.0 / float(self.rate_hz), self.tick)
                self.get_logger().info(f"BNO055 initialized on /dev/i2c-{self.bus_num} addr 0x{self.address:x}")
            except Exception as exc:
                self.get_logger().error(f"Failed to init BNO055: {exc}; running dummy publisher")
                self.timer = self.create_timer(0.5, self.publish_dummy)
        else:
            self.get_logger().warn("smbus2 not available; running dummy publisher")
            self.timer = self.create_timer(0.5, self.publish_dummy)

    # --- Hardware helpers ---
    def _write8(self, reg: int, value: int):
        if not self.bus:
            return
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def _read8(self, reg: int) -> int:
        if not self.bus:
            return 0
        return self.bus.read_byte_data(self.address, reg)

    def _read_block(self, reg: int, length: int) -> bytes:
        if not self.bus:
            return bytes()
        return bytes(self.bus.read_i2c_block_data(self.address, reg, length))

    def _set_mode(self, mode: int):
        self._write8(BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.02)

    def _initialize_sensor(self):
        # reset
        self._set_mode(MODE_CONFIG)
        self._write8(BNO055_SYS_TRIGGER_ADDR, 0x20)
        time.sleep(0.65)
        # normal power, units default
        self._write8(BNO055_PWR_MODE_ADDR, POWER_NORMAL)
        self._write8(BNO055_PAGE_ID_ADDR, 0)
        self._write8(BNO055_UNIT_SEL_ADDR, 0x00)
        time.sleep(0.02)
        # NDOF fusion mode
        self._set_mode(MODE_NDOF)
        time.sleep(0.02)

    # --- Callbacks ---
    def handle_reset(self, request, response):
        try:
            self._set_mode(MODE_CONFIG)
            self._write8(BNO055_SYS_TRIGGER_ADDR, 0x20)
            time.sleep(0.65)
            self._initialize_sensor()
            response.success = True
            response.message = "reset ok"
        except Exception as exc:  # pragma: no cover
            response.success = False
            response.message = f"reset failed: {exc}"
        return response

    def handle_calibrate(self, request, response):
        # Placeholder: real calibration requires motion; we just report status
        try:
            status = self._read8(BNO055_CALIB_STAT_ADDR)
            response.success = True
            response.message = f"calib=0x{status:02x}"
        except Exception as exc:
            response.success = False
            response.message = f"calibration read failed: {exc}"
        return response

    def tick(self):
        try:
            quat_raw = self._read_block(BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
            if len(quat_raw) != 8:
                return
            w, x, y, z = struct.unpack('<hhhh', quat_raw)
            scale = 1.0 / (1 << 14)
            qw, qx, qy, qz = [c * scale for c in (w, x, y, z)]

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation_covariance[0] = -1.0  # Unknown covariance

            # Angular velocity / linear accel not read here; set to 0
            self.pub_imu.publish(imu_msg)

            temp_raw = self._read8(BNO055_TEMP_ADDR)
            temp_msg = Temperature()
            temp_msg.header = imu_msg.header
            temp_msg.temperature = float(temp_raw)
            self.pub_temp.publish(temp_msg)

            # Magnetic field not provided in this lightweight port; publish zeros
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            self.pub_mag.publish(mag_msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"IMU read error: {exc}")

    def publish_dummy(self):
        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.w = 1.0
        self.pub_imu.publish(imu_msg)

        temp_msg = Temperature()
        temp_msg.header = now
        temp_msg.temperature = 25.0
        self.pub_temp.publish(temp_msg)

        mag_msg = MagneticField()
        mag_msg.header = now
        self.pub_mag.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bno055Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
