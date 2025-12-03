#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty

msg = """
Control Your TurtleBot4!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
d   : undock
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/turtlebot4_control/cmd_vel', 10)
        self.undock_client = ActionClient(self, Undock, '/client/undock')
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.settings = termios.tcgetattr(sys.stdin)
        
        print(msg)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = getKey(self.settings)
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            print(f"currently:\tspeed {self.speed}\tturn {self.turn}")
            if (self.status == 14):
                print(msg)
            self.status = (self.status + 1) % 15
        elif key == 'd':
            # Undock
            self.get_logger().info("Undocking...")
            goal_msg = Undock.Goal()
            
            if not self.undock_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("Undock action server not ready")
            else:
                self.get_logger().info("Sending Undock goal...")
                self.future = self.undock_client.send_goal_async(goal_msg)
                self.future.add_done_callback(self.goal_response_callback)

        elif key == ' ' or key == 'k':
            self.x = 0.0
            self.th = 0.0
        elif key == '\x03':
            rclpy.shutdown()
            return

        else:
            self.x = 0.0
            self.th = 0.0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = self.x * self.speed
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = self.th * self.turn
        self.publisher_.publish(twist)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: is_docked = {result.is_docked}')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        # rclpy.shutdown() # Handled in loop

if __name__ == '__main__':
    main()
