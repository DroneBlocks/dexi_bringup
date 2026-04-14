"""
Lightweight node that declares DEXI platform parameters for the web dashboard.
Parameters are set via launch file and queryable via rosapi/roslib.
"""
import rclpy
from rclpy.node import Node


class PlatformParamsNode(Node):
    def __init__(self):
        super().__init__('dexi_platform_params')
        self.declare_parameter('dexi_platform', 'unknown')
        self.declare_parameter('dexi_keyboard_control', False)

        platform = self.get_parameter('dexi_platform').value
        keyboard = self.get_parameter('dexi_keyboard_control').value
        self.get_logger().info(f'Platform: {platform}, Keyboard control: {keyboard}')


def main(args=None):
    rclpy.init(args=args)
    node = PlatformParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
