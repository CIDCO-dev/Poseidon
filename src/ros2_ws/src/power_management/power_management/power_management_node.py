import subprocess

import rclpy
from rclpy.node import Node

from i2c_controller_service.srv import I2cControllerService


class PowerManagementNode(Node):
    """
    ROS2 port of power_management: monitors voltage via i2c_controller_service and triggers shutdown below threshold.
    """

    def __init__(self):
        super().__init__("power_management")
        self.threshold = float(self.declare_parameter("shutdown_voltage", 11.0).value)
        self._shutdown_triggered = False
        self.i2c_client = self.create_client(I2cControllerService, "i2c_controller_service")
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f"power_management ready, threshold={self.threshold}V")

    def tick(self):
        if self._shutdown_triggered:
            return
        if not self.i2c_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("i2c_controller_service unavailable")
            return
        req = I2cControllerService.Request()
        req.action2perform = "get_voltage"
        future = self.i2c_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("get_voltage service call failed")
            return
        voltage = future.result().value
        if voltage <= 0:
            return
        if voltage < self.threshold:
            self.get_logger().warn("Voltage low (%.2f V), launching shutdown procedure" % voltage)
            self._shutdown_triggered = True
            self._graceful_shutdown()

    def _graceful_shutdown(self):
        try:
            subprocess.call(["systemctl", "stop", "ros"])
        except Exception as exc:
            self.get_logger().error(f"Failed to stop ros service: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = PowerManagementNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
