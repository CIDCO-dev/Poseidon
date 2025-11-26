import rclpy
from .logger_text import LoggerText


def main(args=None):
    rclpy.init(args=args)
    output = "/opt/Poseidon/www/webroot/record"
    node = LoggerText(output_folder=output)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
