import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class TimeVerifier(Node):
    def __init__(self):
        super().__init__('time_verifier')
        self.sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.last_time = 0.0

    def clock_callback(self, msg):
        current_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if current_time > self.last_time:
            self.get_logger().info(f"Received Clock: {current_time:.2f}")
            self.last_time = current_time

def main():
    rclpy.init()
    node = TimeVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
