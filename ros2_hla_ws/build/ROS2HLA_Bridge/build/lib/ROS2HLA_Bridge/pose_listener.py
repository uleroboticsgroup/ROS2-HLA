import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.subscription1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.listener_callback2,
            10)
        self.get_logger().info('Pose Listener started. Listening to /turtle1/pose and /turtle2/pose')

    def listener_callback1(self, msg):
        self.get_logger().info(f'Turtle1: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

    def listener_callback2(self, msg):
        self.get_logger().info(f'Turtle2: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    pose_listener = PoseListener()
    rclpy.spin(pose_listener)
    pose_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
