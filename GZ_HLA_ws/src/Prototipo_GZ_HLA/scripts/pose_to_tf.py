#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        
        self.declare_parameter('pose_topic', '/robot_1/odometry')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        
        self.pose_topic = self.get_parameter('pose_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        self.subscription = self.create_subscription(
            Odometry,
            self.pose_topic,
            self.pose_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        
        self.get_logger().info(f'Broadcasting TF {self.parent_frame} -> {self.child_frame} from {self.pose_topic}')

    def pose_callback(self, msg):
        # Extract pose from Odometry
        pose = msg.pose.pose
        self.get_logger().info(f"Received pose: {pose.position.x:.2f}, {pose.position.y:.2f}")
        # Broadcast TF
        t = TransformStamped()
        # Use Time(seconds=0) to indicate "latest available transform"
        # This avoids issues with clock synchronization between Gazebo/HLA/ROS2
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)
        
        # Publish Marker
        marker = Marker()
        marker.header.frame_id = self.parent_frame
        marker.header.stamp = t.header.stamp # Match TF stamp
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        # Offset marker to match chassis center if needed, but msg is chassis pose
        # Chassis size: 2.01142 1 0.568726
        marker.scale.x = 2.0
        marker.scale.y = 1.0
        marker.scale.z = 0.57
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
