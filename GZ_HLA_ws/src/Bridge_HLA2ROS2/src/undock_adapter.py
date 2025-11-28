import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from irobot_create_msgs.action import Undock

class UndockAdapter(Node):
    def __init__(self):
        super().__init__('undock_adapter')
        self.trigger_sub = self.create_subscription(Bool, '/hla_undock_trigger', self.trigger_callback, 10)
        self._action_client = ActionClient(self, Undock, '/undock')
        self.get_logger().info("Undock Adapter Ready. Waiting for /hla_undock_trigger...")

    def trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received Undock Trigger! Sending Action Goal...")
            self.send_goal()

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = Undock.Goal()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Undocking...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Undock Finished!')

def main(args=None):
    rclpy.init(args=args)
    node = UndockAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
