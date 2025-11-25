import rclpy
from .logger_node import LoggerNode


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode(node_name='logger_text_node')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
