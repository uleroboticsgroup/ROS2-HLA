#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/turtlebot4_control/odom_monitor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Monitor Node Started. Waiting for Odometry data...')

    def listener_callback(self, msg):
        # Clear line and print status
        sys.stdout.write(f"\r\033[KReceived Odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print() # Newline on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
