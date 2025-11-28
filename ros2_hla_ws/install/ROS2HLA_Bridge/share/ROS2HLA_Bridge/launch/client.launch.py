from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    config_file = os.path.join(pkg_share, 'config', 'client_config.yaml')
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='Turtle1',
        description='Name of the robot to control'
    )
    
    return LaunchDescription([
        robot_name_arg,
        Node(
            package='ROS2HLA_Bridge',
            executable='bridge_node',
            name='client_bridge',
            parameters=[
                {'config_file': config_file},
                {'robot_name': LaunchConfiguration('robot_name')}
            ]
        )
    ])
