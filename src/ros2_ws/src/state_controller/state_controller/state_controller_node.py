import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PointStamped
from raspberrypi_vitals_msg.msg import sysinfo as SysInfo
from state_controller_msg.msg import State
from state_controller_msg.srv import GetStateService


class StateController(Node):
    def __init__(self) -> None:
        super().__init__('stateControl')
        self.state = State()
        self.state.position.status.status = -1
        self._lock = threading.Lock()

        self.position_sub = self.create_subscription(NavSatFix, 'fix', self.gnss_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.sonar_sub = self.create_subscription(PointStamped, 'depth', self.sonar_callback, 10)
        self.vitals_sub = self.create_subscription(SysInfo, 'vitals', self.vitals_callback, 10)

        self.state_pub = self.create_publisher(State, 'state', 10)
        self.get_state_srv = self.create_service(GetStateService, 'get_state', self.handle_get_state)

        self.get_logger().info('stateControl node initialized (ROS2)')

    def _update_and_publish(self):
        with self._lock:
            msg = self.state
        self.state_pub.publish(msg)

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.service > 0:
            with self._lock:
                self.state.position = msg
                self.state.stamp = msg.header.stamp
            self._update_and_publish()

    def imu_callback(self, msg: Imu):
        with self._lock:
            self.state.imu = msg
            self.state.stamp = msg.header.stamp
        self._update_and_publish()

    def sonar_callback(self, msg: PointStamped):
        with self._lock:
            self.state.depth = msg
            self.state.stamp = msg.header.stamp
        self._update_and_publish()

    def vitals_callback(self, msg: SysInfo):
        with self._lock:
            self.state.vitals = msg
            self.state.stamp = msg.header.stamp
        self._update_and_publish()

    def handle_get_state(self, request, response):
        with self._lock:
            response.state = self.state
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StateController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
