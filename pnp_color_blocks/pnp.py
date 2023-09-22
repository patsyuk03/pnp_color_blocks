import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PnPBlocks(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Hello World PNP!')

    def timer_callback(self):
        self.get_logger().info('Hello World PNP!')

def main(args=None):
    rclpy.init(args=args)
    pnp_blocks = PnPBlocks()
    rclpy.spin(pnp_blocks)
    pnp_blocks.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()