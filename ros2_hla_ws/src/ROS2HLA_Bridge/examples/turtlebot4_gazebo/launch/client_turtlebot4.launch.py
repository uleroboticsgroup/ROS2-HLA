from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    config_file = os.path.join(pkg_share, 'examples', 'turtlebot4_gazebo', 'config', 'client_config.yaml')
    
    bridge_node = Node(
        package='ROS2HLA_Bridge',
        executable='bridge_node',
        name='client_bridge',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'robot_name': 'TurtleBot4'}
        ]
    )
    
    control_node = Node(
        package='ROS2HLA_Bridge',
        executable='control_node.py',
        name='client_control',
        output='screen',
        prefix='gnome-terminal --'
    )
    
    monitor_node = Node(
        package='ROS2HLA_Bridge',
        executable='monitor_node.py',
        name='client_monitor',
        output='screen'
    )

    return LaunchDescription([
        bridge_node,
        control_node,
        monitor_node
    ])
