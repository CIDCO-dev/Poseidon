import enum
import rclpy
from rclpy.node import Node

from logger_interfaces.srv import (
    GetLoggingStatus,
    ToggleLogging,
    GetLoggingMode,
    SetLoggingMode,
    TriggerTransfer,
)


class LoggingMode(enum.IntEnum):
    UNDEFINED = 0
    ALWAYS_ON = 1
    MANUAL = 2
    SPEED_BASED = 3


class LoggerNode(Node):
    def __init__(self, node_name: str = 'logger') -> None:
        super().__init__(node_name)
        self.logging_enabled = False
        self.logging_mode = self.declare_parameter('logging_mode', LoggingMode.ALWAYS_ON.value).value

        # Services mirroring the ROS1 logger API
        self.srv_toggle = self.create_service(ToggleLogging, 'toggle_logging', self.handle_toggle_logging)
        self.srv_status = self.create_service(GetLoggingStatus, 'get_logging_status', self.handle_get_logging_status)
        self.srv_get_mode = self.create_service(GetLoggingMode, 'get_logging_mode', self.handle_get_logging_mode)
        self.srv_set_mode = self.create_service(SetLoggingMode, 'set_logging_mode', self.handle_set_logging_mode)
        self.srv_trigger_transfer = self.create_service(TriggerTransfer, 'trigger_transfer', self.handle_trigger_transfer)

        self.get_logger().info(
            f'LoggerNode ready (mode={self.logging_mode}, enabled={self.logging_enabled})'
        )

    def handle_toggle_logging(self, request: ToggleLogging.Request, response: ToggleLogging.Response):
        self.logging_enabled = bool(request.logging_enabled)
        response.logging_status = self.logging_enabled
        self.get_logger().info(f"Logging toggled -> enabled={self.logging_enabled}")
        return response

    def handle_get_logging_status(self, request: GetLoggingStatus.Request, response: GetLoggingStatus.Response):
        response.status = self.logging_enabled
        return response

    def handle_get_logging_mode(self, request: GetLoggingMode.Request, response: GetLoggingMode.Response):
        response.logging_mode = int(self.logging_mode)
        return response

    def handle_set_logging_mode(self, request: SetLoggingMode.Request, response: SetLoggingMode.Response):
        if request.logging_mode in [mode.value for mode in LoggingMode]:
            self.logging_mode = request.logging_mode
            self.get_logger().info(f"Logging mode set to {self.logging_mode}")
        else:
            self.get_logger().warn(
                f"Ignoring invalid logging mode {request.logging_mode}; keeping {self.logging_mode}"
            )
        return response

    def handle_trigger_transfer(self, request: TriggerTransfer.Request, response: TriggerTransfer.Response):
        # Placeholder: the real implementation should perform packaging/upload.
        response.success = True
        response.message = 'Transfer stub executed (no-op)'
        self.get_logger().info('TriggerTransfer called (stub)')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
