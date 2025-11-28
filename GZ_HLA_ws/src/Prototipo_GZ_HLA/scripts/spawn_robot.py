#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import argparse

class RobotSpawner(Node):
    def __init__(self, name, x, y, z):
        super().__init__('robot_spawner')
        self.client = self.create_client(SpawnEntity, '/world/empty/create')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.name = name
        self.x = x
        self.y = y
        self.z = z

    def spawn(self):
        self.req.entity_factory.name = self.name
        self.req.entity_factory.pose.position.x = float(self.x)
        self.req.entity_factory.pose.position.y = float(self.y)
        self.req.entity_factory.pose.position.z = float(self.z)
        self.req.entity_factory.sdf = f"""
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name='{self.name}'>
    <pose>0 0 0 0 -0 0</pose>
    <link name='chassis'>
      <pose>-0.151427 -0 0.175 0 -0 0</pose>
      <inertial>
        <mass>1.14395</mass>
        <inertia>
          <ixx>0.126164</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.416519</iyy>
          <iyz>0</iyz>
          <izz>0.481014</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry>
          <box>
            <size>2.01142 1 0.568726</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 1.0 1</ambient>
          <diffuse>0.5 0.5 1.0 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box>
            <size>2.01142 1 0.568726</size>
          </box>
        </geometry>
      </collision>
    </link>
    <link name='left_wheel'>
      <pose>0.554283 0.625029 -0.025 -1.5707 0 0</pose>
      <inertial>
        <mass>2</mass>
        <inertia>
          <ixx>0.145833</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.145833</iyy>
          <iyz>0</iyz>
          <izz>0.125</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <link name='right_wheel'>
      <pose>0.554282 -0.625029 -0.025 -1.5707 0 0</pose>
      <inertial>
        <mass>2</mass>
        <inertia>
          <ixx>0.145833</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.145833</iyy>
          <iyz>0</iyz>
          <izz>0.125</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.3</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <link name='caster'>
      <pose>-0.957138 -0 -0.125 0 -0 0</pose>
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.2</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.2</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name='left_wheel_joint' type='revolute'>
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <joint name='right_wheel_joint' type='revolute'>
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <joint name='caster_wheel' type='ball'>
      <parent>chassis</parent>
      <child>caster</child>
    </joint>
    <plugin name='gz::sim::systems::DiffDrive' filename='gz-sim-diff-drive-system'>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>1.25</wheel_separation>
      <wheel_radius>0.3</wheel_radius>
      <odom_publish_frequency>1</odom_publish_frequency>
      <topic>cmd_vel</topic>
    </plugin>
  </model>
</sdf>
"""
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='robot_1')
    parser.add_argument('--x', default=0.0)
    parser.add_argument('--y', default=0.0)
    parser.add_argument('--z', default=2.0)
    
    # Filter out ROS args
    filtered_args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    parsed_args, _ = parser.parse_known_args(filtered_args)
    
    spawner = RobotSpawner(parsed_args.name, parsed_args.x, parsed_args.y, parsed_args.z)
    spawner.spawn()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
