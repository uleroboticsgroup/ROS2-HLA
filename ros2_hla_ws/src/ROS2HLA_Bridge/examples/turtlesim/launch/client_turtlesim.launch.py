from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    config_file = os.path.join(pkg_share, 'examples', 'turtlesim', 'config', 'client_config.yaml')
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='Turtle1',
        description='Name of the robot to control'
    )
    
    pose_listener_node = Node(
        package='ROS2HLA_Bridge',
        executable='pose_listener.py',
        name='client_pose_listener',
        output='screen',
        parameters=[{'topic_prefix': '/Turtle'}]
    )

    delayed_pose_listener = TimerAction(
        period=3.0,
        actions=[pose_listener_node]
    )

    bridge_node = Node(
        package='ROS2HLA_Bridge',
        executable='bridge_node',
        name='client_bridge',
        parameters=[
            {'config_file': config_file},
            {'robot_name': LaunchConfiguration('robot_name')}
        ]
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='client_turtlesim',
        namespace=['/client/', LaunchConfiguration('robot_name')]
    )

    visualizer_node = Node(
        package='ROS2HLA_Bridge',
        executable='visualizer.py',
        name='client_visualizer',
        parameters=[{
            'pose_topic': ['/', LaunchConfiguration('robot_name'), '/pose'],
            'turtle_name': ['client/', LaunchConfiguration('robot_name'), '/turtle1']
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        turtlesim_node,
        visualizer_node,
        bridge_node,
        delayed_pose_listener
    ])
