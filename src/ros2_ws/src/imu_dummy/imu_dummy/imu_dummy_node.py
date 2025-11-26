import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler


class ImuDummyNode(Node):
    """
    ROS2 port of the imu_dummy: publishes oscillating IMU orientations at 200 Hz.
    """

    def __init__(self):
        super().__init__("imu")
        self.pub = self.create_publisher(Imu, "imu/data", 10)
        self.sequence = 0
        self.timer = self.create_timer(1.0 / 200.0, self.tick)

    def tick(self):
        self.sequence += 1
        heading = math.sin(self.sequence / math.pi) * 30.0
        pitch = math.cos(self.sequence / math.pi) * 20.0
        roll = math.sin(self.sequence / math.pi) * 10.0

        q = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))

        msg = Imu()
        msg.header.seq = self.sequence
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.orientation_covariance[0] = -1.0  # unknown covariance
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDummyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
