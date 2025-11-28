from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ROS2HLA_Bridge')
    config_file = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    
    spawn_turtle2 = ExecuteProcess(
        cmd=[[
            'ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"'
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=turtlesim_node,
                on_start=[spawn_turtle2]
            )
        ),
        Node(
            package='ROS2HLA_Bridge',
            executable='bridge_node',
            name='ros2_hla_bridge',
            parameters=[
                {'config_file': config_file}
            ]
        )
    ])
