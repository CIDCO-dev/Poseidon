import threading
import rclpy
from rclpy.node import Node

from i2c_controller_service.srv import I2cControllerService
from logger_interfaces.srv import GetLoggingStatus


class I2cControllerNode(Node):
    def __init__(self):
        super().__init__('i2c_controller')
        self.srv = self.create_service(I2cControllerService, 'i2c_controller_service', self.handle_request)
        self.logger_status = self.create_client(GetLoggingStatus, 'get_logging_status')
        self.board_version = self.declare_parameter('board_version', 2.0).value
        self._lock = threading.Lock()

    def handle_request(self, request, response):
        # Stub: return -555.0 for unknown actions, 0 for led state.
        action = request.action2perform
        if action == 'get_led_state':
            response.value = 0.0
        elif action.startswith('get_'):
            response.value = -555.0
        elif action in ('led_error', 'led_warning', 'led_ready', 'led_recording', 'led_nofix'):
            response.value = 0.0
        else:
            response.value = -555.0
        return response


def main(args=None):
    rclpy.init(args=args)
    node = I2cControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
