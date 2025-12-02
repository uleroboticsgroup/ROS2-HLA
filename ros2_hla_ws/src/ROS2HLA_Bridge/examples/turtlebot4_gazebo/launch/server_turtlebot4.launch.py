from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    # Config file path matches where we will install it in setup.py
    config_file = os.path.join(pkg_share, 'examples', 'turtlebot4_gazebo', 'config', 'bridge_config.yaml')
    
    # Include TurtleBot4 Gazebo launch
    turtlebot4_gz_pkg = get_package_share_directory('turtlebot4_gz_bringup')
    turtlebot4_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_gz_pkg, 'launch', 'turtlebot4_gz.launch.py')
        )
    )

    bridge_node = Node(
        package='ROS2HLA_Bridge',
        executable='bridge_node',
        name='server_bridge',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'robot_name': 'TurtleBot4'}
        ]
    )

    return LaunchDescription([
        turtlebot4_gz_launch,
        bridge_node
    ])
