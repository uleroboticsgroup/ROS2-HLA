import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.pub.publish(msg)
        self.get_logger().info('Published Twist')

def main(args=None):
    rclpy.init(args=args)
    node = TestPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
