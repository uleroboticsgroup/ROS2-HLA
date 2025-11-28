import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_prototipo = get_package_share_directory('Prototipo_GZ_HLA')
    
    # Bridge script path (assuming Bridge_HLA2ROS2 is external to this WS for now, or user will add it later)
    # For now, let's use the absolute path to the bridge script as before, but config from package share.
    bridge_script = '/home/vicen/ISDEFE/Bridge_HLA2ROS2/src/universal_bridge.py'
    server_config = os.path.join(pkg_prototipo, 'config', 'server_config.yaml')

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ROS-GZ Bridge
    # Bridge Clock, Pose, CmdVel for BOTH robots
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/robot_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/robot_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )

    # Spawners
    spawn_robot_1 = Node(
        package='Prototipo_GZ_HLA',
        executable='spawn_robot.py',
        arguments=['--name', 'robot_1', '--x', '0.0', '--y', '0.0', '--z', '2.0'],
        output='screen'
    )

    spawn_robot_2 = Node(
        package='Prototipo_GZ_HLA',
        executable='spawn_robot.py',
        arguments=['--name', 'robot_2', '--x', '2.0', '--y', '2.0', '--z', '2.0'],
        output='screen'
    )

    # HLA Bridge (Server)
    hla_bridge = Node(
        package='bridge_hla2ros2',
        executable='universal_bridge',
        arguments=[server_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        ros_gz_bridge,
        spawn_robot_1,
        spawn_robot_2,
        hla_bridge
    ])
