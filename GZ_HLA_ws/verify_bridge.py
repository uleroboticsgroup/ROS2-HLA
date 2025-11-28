
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import time
import threading
import sys

class Verifier(Node):
    def __init__(self):
        super().__init__('verifier')
        self.pose_received = False
        self.cmd_received = False
        
        # Subscriber for Client Pose (from Server)
        self.create_subscription(Pose, '/robot_1/pose', self.pose_callback, 10)
        
        # Subscriber for Server Cmd (from Client)
        self.create_subscription(Twist, '/model/robot_1/cmd_vel', self.cmd_callback, 10)
        
        # Publisher for Client Cmd (to Server)
        self.cmd_pub = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        
        self.timer = self.create_timer(1.0, self.publish_cmd)
        
    def pose_callback(self, msg):
        if not self.pose_received:
            self.get_logger().info(f"SUCCESS: Received Pose: {msg.position.x}, {msg.position.y}, {msg.position.z}")
            self.pose_received = True

    def cmd_callback(self, msg):
        if not self.cmd_received:
            self.get_logger().info(f"SUCCESS: Received Cmd on Server Side: {msg.linear.x}")
            self.cmd_received = True
            
    def publish_cmd(self, *args):
        msg = Twist()
        msg.linear.x = 1.0
        self.cmd_pub.publish(msg)
        # self.get_logger().info("Published Cmd on Client Side")

def main():
    rclpy.init()
    node = Verifier()
    
    try:
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose_received and node.cmd_received:
                print("VERIFICATION PASSED: Both Pose and Cmd bridged successfully!")
                break
            
            if time.time() - start_time > 30:
                print("VERIFICATION FAILED: Timeout waiting for data.")
                if not node.pose_received:
                    print("- Pose NOT received (Server -> Client failed)")
                if not node.cmd_received:
                    print("- Cmd NOT received (Client -> Server failed)")
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
