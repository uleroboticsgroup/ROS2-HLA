from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    # Config file path matches where we will install it in setup.py
    config_file = os.path.join(pkg_share, 'examples', 'webots', 'config', 'bridge_config.yaml')
    
    # Note: We assume the user launches Webots separately or we could include a webots launch here
    # For now, we just launch the bridge configured for Webots

    bridge_node = Node(
        package='ROS2HLA_Bridge',
        executable='bridge_node',
        name='server_bridge_webots',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'robot_name': 'WebotsRobot'}
        ]
    )

    return LaunchDescription([
        bridge_node
    ])
