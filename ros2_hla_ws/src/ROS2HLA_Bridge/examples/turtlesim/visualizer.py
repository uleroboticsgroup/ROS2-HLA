#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from functools import partial

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        
        self.declare_parameter('pose_topic', '/turtle1/pose')
        self.declare_parameter('turtle_name', 'turtle1')
        
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        
        self.get_logger().info(f"Visualizer started. Listening to {self.pose_topic}, controlling {self.turtle_name}")
        
        # Subscriber to Pose
        self.create_subscription(Pose, self.pose_topic, self.pose_callback, 10)
        
        # Client for TeleportAbsolute
        self.teleport_client = self.create_client(TeleportAbsolute, f'{self.turtle_name}/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
            
    def pose_callback(self, msg):
        req = TeleportAbsolute.Request()
        req.x = msg.x
        req.y = msg.y
        req.theta = msg.theta
        
        future = self.teleport_client.call_async(req)
        # We don't wait for result to avoid blocking, fire and forget for viz is usually fine
        # or add a done callback if needed.

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
