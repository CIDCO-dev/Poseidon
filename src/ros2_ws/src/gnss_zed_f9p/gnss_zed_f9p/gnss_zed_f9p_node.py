import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gnss_status_msg.msg import GnssDiagnostic


class GnssZedF9PNode(Node):
    def __init__(self):
        super().__init__('zedf9p')
        self.speed_pub = self.create_publisher(Odometry, 'speed', 10)
        self.status_pub = self.create_publisher(GnssDiagnostic, 'gnss_status', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.seq = 0

    def tick(self):
        self.seq += 1
        # Publish a dummy speed from a sine wave
        speed = math.fabs(math.sin(self.seq / 10.0)) * 5.0  # km/h
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.twist.twist.linear.y = speed
        self.speed_pub.publish(odom)

        status = GnssDiagnostic()
        status.header.stamp = odom.header.stamp
        status.fix_type = 3
        status.diff_soln = True
        status.carr_soln = 2
        status.num_sv = 16
        status.horizontal_accuracy = 0.5
        status.vertical_accuracy = 0.8
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = GnssZedF9PNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
