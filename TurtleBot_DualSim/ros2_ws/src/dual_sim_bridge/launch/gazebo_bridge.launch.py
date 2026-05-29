"""
Launch file para el puente DualSim del lado Gazebo.
Lanza la simulación de TurtleBot3 en Gazebo + el bridge HLA.

Uso:
  # Mundo vacío (por defecto)
  ros2 launch dual_sim_bridge gazebo_bridge.launch.py

  # Mundo con dos habitaciones
  ros2 launch dual_sim_bridge gazebo_bridge.launch.py world:=dual_rooms

  # Mundo TurtleBot3 House
  ros2 launch dual_sim_bridge gazebo_bridge.launch.py world:=turtlebot3_house

  # Mundo custom (ruta completa)
  ros2 launch dual_sim_bridge gazebo_bridge.launch.py world:=/ruta/al/mundo.sdf
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    SetEnvironmentVariable, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def resolve_world(context, *args, **kwargs):
    """Resuelve el nombre del mundo a su ruta y launch file."""
    world_name = LaunchConfiguration('world').perform(context)
    pkg_share = get_package_share_directory('dual_sim_bridge')

    # Mundos custom del paquete dual_sim_bridge
    custom_worlds = {
        'dual_rooms': os.path.join(pkg_share, 'worlds', 'dual_rooms.sdf'),
    }

    # Mundos de turtlebot3_gazebo
    try:
        tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
        tb3_worlds = {
            'empty_world': os.path.join(tb3_pkg, 'worlds', 'empty_world.world'),
            'turtlebot3_house': os.path.join(tb3_pkg, 'worlds', 'turtlebot3_house.world'),
            'turtlebot3_world': os.path.join(tb3_pkg, 'worlds', 'turtlebot3_world.world'),
        }
    except Exception:
        tb3_worlds = {}

    # Buscar el mundo
    all_worlds = {**tb3_worlds, **custom_worlds}

    if world_name in all_worlds:
        world_path = all_worlds[world_name]
    elif os.path.isfile(world_name):
        # Ruta absoluta directa
        world_path = world_name
    else:
        print(f"[DualSim] Mundo '{world_name}' no encontrado. Mundos disponibles:")
        for name, path in all_worlds.items():
            exists = "✓" if os.path.isfile(path) else "✗"
            print(f"  {exists} {name}: {path}")
        print(f"  Usando 'empty_world' como fallback.")
        world_path = tb3_worlds.get(
            'empty_world',
            os.path.join(tb3_pkg, 'worlds', 'empty_world.world')
        )

    print(f"[DualSim] Cargando mundo: {world_path}")

    # Convertir SDF a JSON para que Unity reconstruya la geometría
    try:
        from dual_sim_bridge.sdf_to_json import sdf_to_json
        json_path = sdf_to_json(world_path, '/tmp/dualsim_world.json')
        print(f"[DualSim] Geometría exportada a {json_path}")
    except Exception as e:
        print(f"[DualSim] Warning: no se pudo convertir SDF a JSON: {e}")
        # Fallback: escribir un JSON mínimo con solo el nombre
        import json
        fallback = {'world_name': world_name, 'model_count': 0, 'models': []}
        with open('/tmp/dualsim_world.json', 'w') as f:
            json.dump(fallback, f)

    # Posición de spawn del robot según el mundo
    spawn_positions = {
        'dual_rooms': ('-3.0', '0.0'),      # Centro de Room A
        'empty_world': ('0.0', '0.0'),
        'turtlebot3_house': ('-3.0', '1.0'),
        'turtlebot3_world': ('-2.0', '-0.5'),
    }
    x_pose, y_pose = spawn_positions.get(world_name, ('0.0', '0.0'))

    # ── Gazebo Server ──
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_path}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── Gazebo GUI ──
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # ── TurtleBot3 spawn + state publisher ──
    try:
        tb3_launch_dir = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'
        )
        robot_state_pub = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
        spawn_tb3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items()
        )
        return [gzserver, gzclient, spawn_tb3, robot_state_pub]
    except Exception:
        return [gzserver, gzclient]


def resolve_bridge_node(context, *args, **kwargs):
    """Crea el nodo bridge con los parámetros de spawn correctos."""
    world_name = LaunchConfiguration('world').perform(context)

    spawn_positions = {
        'dual_rooms': (-3.0, 0.0),
        'empty_world': (0.0, 0.0),
        'turtlebot3_house': (-3.0, 1.0),
        'turtlebot3_world': (-2.0, -0.5),
    }
    sx, sy = spawn_positions.get(world_name, (0.0, 0.0))
    print(f"[DualSim] Spawn offset para bridge: x={sx}, y={sy}")

    bridge_node = Node(
        package='dual_sim_bridge',
        executable='bridge_node',
        name='gazebo_hla_bridge',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'spawn_x': sx,
            'spawn_y': sy,
        }]
    )
    return [bridge_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('dual_sim_bridge')

    # ── Argumentos ──
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'gazebo_bridge.yaml'),
        description='Ruta al archivo de configuración YAML del bridge'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Mundo de Gazebo: empty_world, dual_rooms, '
                    'turtlebot3_house, turtlebot3_world, o ruta absoluta'
    )

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    return LaunchDescription([
        config_arg,
        world_arg,
        set_tb3_model,
        OpaqueFunction(function=resolve_world),
        OpaqueFunction(function=resolve_bridge_node),
    ])
