import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from setting_msg.srv import ConfigurationService


class SonarDummyNode(Node):
    """
    ROS2 port of sonar_dummy: publishes oscillating depth on /depth, reads config once.
    """

    def __init__(self):
        super().__init__("sonar")
        self.pub = self.create_publisher(PointStamped, "depth", 10)
        self.sequence = 0
        self.timer = self.create_timer(1.0, self.tick)
        self.config_client = self.create_client(ConfigurationService, "get_configuration")
        self._config_lock = threading.Lock()
        self.sonar_params = {
            "sonarStartGain": 0x06,
            "sonarRange": 32,
            "sonarAbsorbtion": 0x14,
            "sonarPulseLength": 150,
        }
        self._load_config()

    def _load_config(self):
        if not self.config_client.wait_for_service(timeout_sec=0.5):
            return
        for key in list(self.sonar_params.keys()):
            req = ConfigurationService.Request()
            req.key = key
            future = self.config_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done() and future.result():
                value = future.result().value
                try:
                    with self._config_lock:
                        self.sonar_params[key] = int(value)
                except ValueError:
                    pass

    def tick(self):
        self.sequence += 1
        msg = PointStamped()
        msg.header.seq = self.sequence
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.z = math.sin(self.sequence) * 30.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarDummyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
