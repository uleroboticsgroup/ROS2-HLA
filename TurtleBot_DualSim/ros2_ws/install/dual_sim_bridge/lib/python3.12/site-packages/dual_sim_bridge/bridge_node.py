"""
Nodo bridge ROS2↔HLA para el sistema DualSim.
Publica la pose del TurtleBot de Gazebo a HLA y recibe
la pose del robot de Unity para mostrar un ghost en RViz.
"""

import rclpy
from rclpy.node import Node
import yaml
import math
import jpype
import threading

from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import subprocess

from dual_sim_bridge.hla_manager import HLAManager


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convierte quaternion a ángulo yaw (rotación en Z)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Convierte ángulo yaw a quaternion (solo rotación en Z)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DualSimBridge(Node):
    def __init__(self):
        super().__init__('dual_sim_bridge')

        # Parámetros
        self.declare_parameter('config_file', '')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        config_file = (
            self.get_parameter('config_file')
            .get_parameter_value().string_value
        )
        self.spawn_x = self.get_parameter('spawn_x').get_parameter_value().double_value
        self.spawn_y = self.get_parameter('spawn_y').get_parameter_value().double_value

        if not config_file:
            self.get_logger().error("¡No se proporcionó config_file!")
            return

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # Configuración local
        local_cfg = self.config['local_robot']
        remote_cfg = self.config['remote_robot']
        self.local_instance = local_cfg['hla_instance_name']
        self.remote_instance = remote_cfg['hla_instance_name']
        self.hla_class = self.config['hla_object_class']
        self.hla_attributes = self.config['hla_attributes']
        self.encoding = self.config.get('encoding', 'float32le')

        # HLA Manager
        self.hla_manager = HLAManager(self.config, self.get_logger())
        self.hla_manager.connect()
        self.hla_manager.create_and_join_federation()

        # Time Management
        self.hla_manager.enable_time_management()

        # Sync Points
        sync_points = self.config.get('sync_points', [])
        if sync_points:
            for sp in sync_points:
                if self.hla_manager.is_regulating:
                    self.hla_manager.register_sync_point(sp)
                self.hla_manager.wait_for_sync_point(sp)

        # Publicar nuestro robot
        self.hla_manager.setup_publication(
            self.hla_class, self.hla_attributes, self.local_instance
        )

        # Suscribirse al robot remoto
        self.hla_manager.setup_subscription(self.hla_class, self.hla_attributes)
        self.hla_manager.on_object_update_received = (
            self.on_remote_robot_update
        )

        # ROS2: TF2 Listener para leer la pose sin depender de /odom plugin
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS2: publicar pose del robot remoto
        self.remote_pose_pub = self.create_publisher(
            PoseStamped,
            remote_cfg['pose_topic'],
            10
        )

        # Flag para Gazebo ghost
        self.gz_ghost_spawned = False
        self._tf_success_logged = False

        # TF broadcaster para el ghost
        self.tf_broadcaster = TransformBroadcaster(self)

        # Lock y timer HLA
        self.hla_lock = threading.Lock()
        self.create_timer(0.01, self.hla_loop)

        # Timer para mover ghost en Gazebo a 5Hz (evita saturar subprocesos)
        self._pending_gz_pose = None
        self.create_timer(0.2, self._gz_pose_timer_callback)

        # Datos del robot remoto
        self.remote_pose = None

        self.get_logger().info(
            f"DualSim Bridge inicializado. "
            f"Local: '{self.local_instance}', "
            f"Remoto: '{self.remote_instance}', "
            f"Spawn offset: ({self.spawn_x}, {self.spawn_y})"
        )

    def _poll_and_publish_local_pose(self):
        """Lee el TF local y lo publica a HLA."""
        try:
            # Buscar el tf de odom -> base_link o base_footprint
            target_frame = 'base_footprint' # Típico para TurtleBot3
            t = self.tf_buffer.lookup_transform(
                'odom',
                target_frame,
                rclpy.time.Time()
            )
            pos = t.transform.translation
            orient = t.transform.rotation

            yaw_deg = math.degrees(
                quaternion_to_yaw(orient.x, orient.y, orient.z, orient.w)
            )

            # TF odom->base_footprint es relativo al spawn.
            # Sumar spawn_x/spawn_y para obtener posición absoluta en el mundo.
            abs_x = pos.x + self.spawn_x
            abs_y = pos.y + self.spawn_y

            # Offset de altura: el suelo de Unity está ~0.5m más arriba que el de Gazebo
            # HlaPlayerSender.cs mapea PositionZ -> Unity Y (altura)
            UNITY_GROUND_OFFSET = 0.5
            data_map = {
                'PositionX': abs_x,
                'PositionY': abs_y,
                'RotationY': yaw_deg,
                'PositionZ': pos.z + UNITY_GROUND_OFFSET,
            }

            self.hla_manager.update_object(
                self.local_instance, self.hla_class,
                data_map, self.encoding
            )
            if not self._tf_success_logged:
                self._tf_success_logged = True
                self.get_logger().info(
                    f"TF odom->base_footprint OK. Pose: "
                    f"x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
        except Exception as e:
            # Avisar periódicamente (cada 500 llamadas ≈ 5s a 100Hz)
            if not hasattr(self, '_tf_err_count'):
                self._tf_err_count = 0
            self._tf_err_count += 1
            if self._tf_err_count % 500 == 1:
                self.get_logger().warn(f"TF odom->base_footprint no disponible ({self._tf_err_count}x): {e}")

    def on_remote_robot_update(self, objectInstance, attributeValues):
        """Recibe actualización del robot remoto desde HLA."""
        obj_name = self.hla_manager.get_object_name(objectInstance)
        if not obj_name:
            return

        # Solo procesar si es el robot remoto esperado
        # (ignorar nuestro propio robot)
        if obj_name == self.local_instance:
            return

        try:
            data = {}
            for attr_name in self.hla_attributes:
                key = (self.hla_class, attr_name)
                if key in self.hla_manager.attribute_handles:
                    handle = self.hla_manager.attribute_handles[key]
                    if attributeValues.containsKey(handle):
                        val_bytes = attributeValues.get(handle)
                        val = self.hla_manager._decode_float(
                            val_bytes, self.encoding
                        )
                        data[attr_name] = val

            if data:
                self.remote_pose = data
                self.publish_remote_pose(data)

        except Exception as e:
            self.get_logger().error(f"Error procesando robot remoto: {e}")

    def publish_remote_pose(self, data):
        """Publica la pose del robot remoto como PoseStamped + Marker + TF."""
        now = self.get_clock().now().to_msg()

        pos_x = data.get('PositionX', 0.0)
        pos_y = data.get('PositionY', 0.0)
        pos_z = data.get('PositionZ', 0.0)
        rot_y_deg = data.get('RotationY', 0.0)
        yaw_rad = math.radians(rot_y_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = pos_x
        pose_msg.pose.position.y = pos_y
        pose_msg.pose.position.z = pos_z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.remote_pose_pub.publish(pose_msg)

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = f'{self.remote_instance}_base_link'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = pos_z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Mover en Gazebo a través de gz-transport directo (throttled)
        self._pending_gz_pose = (pos_x, pos_y, pos_z, qx, qy, qz, qw)

        # Spawnea la figura en Gazebo si no lo hemos hecho aún y si es UnityRobot
        if not self.gz_ghost_spawned and self.remote_instance == 'UnityRobot':
            self.gz_ghost_spawned = True
            sdf_str = (
                '<sdf version="1.8"><model name="UnityRobot">'
                '<static>false</static>'
                '<link name="base_link">'
                '<gravity>false</gravity><kinematic>true</kinematic>'
                '<visual name="visual"><geometry><box>'
                '<size>0.3 0.3 0.15</size></box></geometry><material><ambient>0.0 1.0 0.0 1.0</ambient><diffuse>0.0 1.0 0.0 1.0</diffuse>'
                '</material></visual></link></model></sdf>'
            )
            cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default', '-name', 'UnityRobot', '-string', sdf_str]
            subprocess.Popen(cmd)
            import time as _time
            self._gz_spawn_time = _time.time()
            self.get_logger().info("Ghost 'UnityRobot' spawneado en Gazebo. Esperando 2s para mover...")

    def _gz_pose_timer_callback(self):
        """Timer a 5Hz para mover el ghost en Gazebo sin saturar subprocesos."""
        if not hasattr(self, '_pending_gz_pose') or self._pending_gz_pose is None:
            return
        # No mover hasta que el modelo haya sido spawneado
        if not self.gz_ghost_spawned:
            return
        # Esperar 2s después del spawn para que Gazebo registre la entidad
        if not hasattr(self, '_gz_spawn_time'):
            return
        import time as _time
        if _time.time() - self._gz_spawn_time < 2.0:
            return
        
        # Tomar y limpiar el dato pendiente
        pose = self._pending_gz_pose
        self._pending_gz_pose = None
        x, y, z, qx, qy, qz, qw = pose
        model_name = self.remote_instance
        
        req = [
            'gz', 'service',
            '-s', '/world/default/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '200',
            '--req',
            f'name: "{model_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        ]
        subprocess.Popen(req,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def hla_loop(self):
        """Timer callback para procesar callbacks HLA y avanzar el tiempo."""
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()

        with self.hla_lock:
            try:
                # Leer y mandar nuestra pose a la HLA Federation
                self._poll_and_publish_local_pose()

                if (self.hla_manager.is_regulating
                        or self.hla_manager.is_constrained):
                    self.hla_manager.advance_time()
                else:
                    self.hla_manager.spin_once()
            except Exception as e:
                self.get_logger().error(f"Error en HLA loop: {e}")

    def destroy_node(self):
        self.hla_manager.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualSimBridge()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
