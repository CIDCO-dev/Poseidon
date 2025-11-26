import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gnss_status_msg.msg import GnssDiagnostic
from nav_msgs.msg import Odometry


class GnssDummyNode(Node):
    def __init__(self):
        super().__init__('gnss_dummy')
        self.pub_fix = self.create_publisher(NavSatFix, 'fix', 10)
        self.pub_status = self.create_publisher(GnssDiagnostic, 'gnss_status', 10)
        self.pub_speed = self.create_publisher(Odometry, 'speed', 10)
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

        now = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.seq = self.sequence = self.sequence + 1
        fix.header.stamp = now
        fix.status.status = status
        fix.status.service = 1
        fix.longitude = self.longitude
        fix.latitude = self.latitude
        fix.altitude = self.ellipsoidal_height(self.sequence)
        fix.position_covariance_type = 0
        self.pub_fix.publish(fix)

        status_msg = GnssDiagnostic()
        status_msg.header.stamp = now
        status_msg.fix_type = 3 if status >= 0 else 0
        status_msg.diff_soln = status >= 0
        status_msg.carr_soln = 2 if status >= 0 else 0
        status_msg.num_sv = 12
        status_msg.horizontal_accuracy = 1.0 if status >= 0 else 99.0
        status_msg.vertical_accuracy = 1.5 if status >= 0 else 99.0
        self.pub_status.publish(status_msg)

        speed = Odometry()
        speed.header.stamp = now
        speed.twist.twist.linear.y = abs(math.sin(self.sequence / 10.0)) * 5.0
        self.pub_speed.publish(speed)

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
