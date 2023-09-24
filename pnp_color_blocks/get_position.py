import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GetPosition(Node):

    def __init__(self):
        super().__init__('get_position')
        self.get_logger().info('Hello World PNP!')

    def timer_callback(self):
        self.get_logger().info('Hello World PNP!')

def main(args=None):
    rclpy.init(args=args)
    get_position = GetPosition()
    rclpy.spin(get_position)
    get_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()