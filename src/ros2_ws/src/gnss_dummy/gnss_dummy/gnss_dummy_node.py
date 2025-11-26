import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GnssDummyNode(Node):
    def __init__(self):
        super().__init__('gnss')
        self.pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.sequence = 0
        self.longitude = 0.0
        self.latitude = 0.0
        self.timer = self.create_timer(1.0, self.tick)
        self.gnss_fix_change = 0

    def tick(self):
        status = -1 if self.gnss_fix_change < 60 else 1
        if self.gnss_fix_change >= 90:
            self.gnss_fix_change = 30
        else:
            self.gnss_fix_change += 1

        msg = NavSatFix()
        msg.header.seq = self.sequence = self.sequence + 1
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.status = status
        msg.status.service = 1
        msg.longitude = self.longitude
        msg.latitude = self.latitude
        msg.altitude = self.ellipsoidal_height(self.sequence)
        msg.position_covariance_type = 0
        self.pub.publish(msg)

    def ellipsoidal_height(self, seq: int) -> float:
        return math.sin(seq / 10.0) * 10.0


def main(args=None):
    rclpy.init(args=args)
    node = GnssDummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
