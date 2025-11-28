#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

import sys

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        
        self.declare_parameter('topic_prefix', '/turtle')
        prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value
        
        self.subscription1 = self.create_subscription(
            Pose,
            f'{prefix}1/pose',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Pose,
            f'{prefix}2/pose',
            self.listener_callback2,
            10)
        
        self.pose1 = None
        self.pose2 = None
        
        # Print initial newlines to reserve space
        print("\n\n")
        
        self.timer = self.create_timer(0.1, self.print_poses)
        self.get_logger().info('Pose Listener started. Dashboard active.')

    def listener_callback1(self, msg):
        self.pose1 = msg

    def listener_callback2(self, msg):
        self.pose2 = msg

    def print_poses(self):
        # Move cursor up 2 lines
        sys.stdout.write("\033[2A")
        
        # Print Turtle 1
        if self.pose1:
            sys.stdout.write(f"\r\033[KTurtle1: x={self.pose1.x:6.2f}, y={self.pose1.y:6.2f}, theta={self.pose1.theta:6.2f}\n")
        else:
            sys.stdout.write(f"\r\033[KTurtle1: Waiting for data...\n")
            
        # Print Turtle 2
        if self.pose2:
            sys.stdout.write(f"\r\033[KTurtle2: x={self.pose2.x:6.2f}, y={self.pose2.y:6.2f}, theta={self.pose2.theta:6.2f}\n")
        else:
            sys.stdout.write(f"\r\033[KTurtle2: Waiting for data...\n")
        
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    pose_listener = PoseListener()
    try:
        rclpy.spin(pose_listener)
    except KeyboardInterrupt:
        pass
    finally:
        pose_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()