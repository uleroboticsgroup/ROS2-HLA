import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import random

# Note: Using geometry_msgs/Pose because turtlesim/Pose might not be available in standard installs
# But the config uses turtlesim.msg.Pose. 
# If turtlesim is installed, we use it. If not, we fallback?
# I checked and turtlesim is available.

try:
    from turtlesim.msg import Pose
except ImportError:
    from geometry_msgs.msg import Pose

class MockTurtle(Node):
    def __init__(self):
        super().__init__('turtlesim')
        self.pub = self.create_publisher(Pose, '/turtle1/pose', 10)
        self.sub = self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.x = 5.5
        self.y = 5.5

    def timer_callback(self):
        msg = Pose()
        msg.x = self.x
        msg.y = self.y
        # msg.theta = 0.0
        # msg.linear_velocity = 0.0
        # msg.angular_velocity = 0.0
        
        self.x += random.uniform(-0.1, 0.1)
        self.y += random.uniform(-0.1, 0.1)
        self.pub.publish(msg)
        # self.get_logger().info(f'Published Pose: {msg.x:.2f}, {msg.y:.2f}')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received CmdVel: LinearX={msg.linear.x}, AngularZ={msg.angular.z}')

def main():
    rclpy.init()
    node = MockTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
