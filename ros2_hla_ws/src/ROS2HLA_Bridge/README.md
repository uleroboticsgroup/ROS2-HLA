# ROS2HLA_Bridge

Este paquete proporciona un puente genérico entre **ROS2** (Robot Operating System 2) y **HLA** (High Level Architecture, IEEE 1516e) para permitir la simulación colaborativa a través de diferentes entornos.

Aunque el núcleo del puente es agnóstico y configurable para cualquier sistema ROS2, este paquete incluye un ejemplo totalmente funcional utilizando **Turtlesim**, otro ejemplo con **TurtleBot4** y otro con **Webots** para demostrar sus capacidades.

## Características
- **Genérico y Configurable**: Define mapeos entre Topics de ROS2 y Objetos/Interacciones HLA a través de archivos YAML.
- **Puente Bidireccional**: 
    - Publica datos de ROS2 a HLA (Atributos/Interacciones).
    - Se suscribe a datos de HLA y publica en ROS2.
- **Gestión de Tiempo**: 
    - Soporta modos **Time Regulating** (Servidor) y **Time Constrained** (Cliente).
    - **Pacer en Tiempo Real**: Sincroniza el tiempo lógico de HLA con el tiempo del reloj de pared para una visualización fluida en tiempo real.
- **Escalable**: Soporta plantillas para lanzar múltiples federados (por ejemplo, swarms de robots).
- **Visualización del Cliente**: El usuario puede visualizar las entidades remotas, pero debe implementarse individualmente para cada aplicación.

## Prerrequisitos
- ROS2 Jazzy
- Pitch RTI (prti1516e.jar) en `{{pitch_rti_path}}`
- Paquete `turtlesim` (para el ejemplo de Turtlesim).
- Paquete `turtlebot4` (para el ejemplo de TurtleBot4).
- Paquete `webots_ros2_turtlebot` (para el ejemplo de Webots).

## Guía de Configuración

El puente se maneja mediante archivos de configuración YAML que se encuentran en `config/`.

### 1. Configuración HLA
Define la conexión a la RTI.
```yaml
hla:
  federation_name: "RoboticsFed"
  federate_name: "ROS2Bridge"
  fom_file_path: "/path/to/fom.xml"
  pitch_jar_path: "/path/to/prti1516e.jar"
```

### 2. Gestión de Tiempo
Controla cómo avanza el tiempo el federado.
```yaml
time_management:
  is_regulating: true   # Dirige el tiempo de simulación (Servidor)
  is_constrained: false # Obedece al tiempo de simulación (Cliente)
  time_step: 0.1        # Tamaño del paso para el avance de tiempo
  lookahead: 0.1        # Lookahead para federados reguladores
```

### 3. ROS2 -> HLA (Publicar a HLA)
Mapea un topic de ROS2 a un Atributo de Objeto o Interacción HLA.
```yaml
ros_to_hla:
  - ros_topic: "/turtle1/pose"
    ros_type: "turtlesim.msg.Pose"
    hla_object_class: "Robot"
    hla_instance_name: "Turtle1"
    mapping:
      x: "positionX"      # Campo ROS : Atributo HLA
      y: "positionY"
      theta: "orientation"
```

### 4. HLA -> ROS2 (Suscribirse desde HLA)
Mapea un Atributo de Objeto o Interacción HLA a un topic de ROS2.
```yaml
hla_to_ros:
  - ros_topic: "/turtle1/cmd_vel"
    ros_type: "geometry_msgs.msg.Twist"
    hla_interaction_class: "Control"
    filter_parameter: "robotName" # Filtrado opcional
    filter_value: "Turtle1"
    mapping:
      linear.x: "linearVel"
      angular.z: "angularVel"
```

### 5. Plantillas para múltiples federados
Se pueden usar `{{variable_name}}` en el archivo YAML. Estas variables se reemplazan en tiempo de ejecución por argumentos de lanzamiento.
Ejemplo: `ros_topic: "/{{robot_name}}/pose"` permite lanzar el mismo puente para "Turtle1", "Turtle2", etc.

## Ejemplo 1: Simulación Colaborativa con Turtlesim
Este ejemplo demuestra un Servidor (Regulador) ejecutando la simulación física y múltiples Clientes (Restringidos) visualizando y controlando los robots.

### 1. Iniciar el Servidor (Federado Regulador)
Lanza la simulación principal y el puente.
```bash
ros2 launch ROS2HLA_Bridge server_turtlesim.launch.py
```

### 2. Iniciar un Cliente (Federado Restringido)
Lanza un puente cliente y un visualizador local.
```bash
ros2 launch ROS2HLA_Bridge client_turtlesim.launch.py robot_name:=Turtle1
```

### 3. Controlar el Robot
El cliente expone topics bajo el espacio de nombres `/client/<robot_name>/`.

Para mover a `Turtle1`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Turtle1/cmd_vel
```

### Arquitectura (Ejemplo Turtlesim)

#### Servidor
- **Nodo Turtlesim**: La simulación física.
- **Nodo Bridge**: Publica `/turtleX/pose` -> Objeto HLA `Robot`. Se suscribe Interacción HLA `Control` -> `/turtleX/cmd_vel`.

#### Cliente
- **Nodo Bridge**: Se suscribe Objeto HLA `Robot` -> `/TurtleX/pose`. Publica `/TurtleX/cmd_vel` -> Interacción HLA `Control`.
- **Nodo Turtlesim (Visualizador)**: Una simulación "fantasma" que solo muestra la tortuga.
- **Nodo Visualizador**: Se suscribe a `/TurtleX/pose` y llama a `/TurtleX/teleport_absolute` para actualizar el visualizador.

## Ejemplo 2: Simulación TurtleBot4 en Gazebo
Este ejemplo demuestra el control de un TurtleBot4 en una simulación de Gazebo vía HLA.

### 1. Iniciar el Servidor (Federado Regulador)
Lanza la simulación de Gazebo (mundo Warehouse) y el puente.
```bash
ros2 launch ROS2HLA_Bridge server_turtlebot4.launch.py
```

### 2. Iniciar el Cliente (Federado Restringido)
Lanza el puente cliente y un controlador teleop interactivo en la misma terminal.
```bash
ros2 run ROS2HLA_Bridge client_interactive
```

### 3. Controlar el Robot
Usa las teclas del teclado (u, i, o, j, k, l, m, ,, .) en la terminal del cliente para mover el robot.
- **i**: Avanzar
- **k**: Parar
- **j**: Izquierda
- **l**: Derecha
- **espacio**: Parada forzosa
- **d**: Desacoplar (Undock)
- **f**: Acoplar (Dock)

## Ejemplo 3: Simulación Webots
Este ejemplo demuestra el control de un robot en una simulación de Webots vía HLA.

### 1. Prerrequisitos
Asegúrate de tener instalado el driver ROS2 de Webots (ej. `webots_ros2_turtlebot`).

### 2. Iniciar Webots
Lanza tu simulación de Webots.
```bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

### 3. Iniciar el Servidor (Federado Regulador)
Lanza el puente configurado para Webots.
```bash
ros2 launch ROS2HLA_Bridge server_webots.launch.py
```

### 4. Iniciar el Cliente (Federado Restringido)
Lanza el puente cliente y un controlador teleop interactivo en la misma terminal.
```bash
ros2 run ROS2HLA_Bridge client_interactive
```

---

# ROS2HLA_Bridge

This package provides a generic bridge between **ROS2** (Robot Operating System 2) and **HLA** (High Level Architecture, IEEE 1516e) to enable collaborative simulation across different environments.

While the core bridge is agnostic and configurable for any ROS2 system, this package includes a fully functional example using **Turtlesim**, **TurtleBot4** and **Webots** to demonstrate its capabilities.

## Features
- **Generic & Configurable**: Define mappings between ROS2 Topics and HLA Objects/Interactions via YAML files.
- **Bidirectional Bridging**: 
    - Publish ROS2 data to HLA (Attributes/Interactions).
    - Subscribe to HLA data and publish to ROS2.
- **Time Management**: 
    - Supports **Time Regulating** (Server) and **Time Constrained** (Client) modes.
    - **Real-Time Pacer**: Synchronizes the HLA logical time with wall-clock time for smooth, real-time visualization.
- **Scalable**: Supports templating for launching multiple federates (e.g., swarms of robots).
- **Client Visualization**: The user can visualize the remote entities, but it needs to be implemented individually for each application.

## Prerequisites
- ROS2 Jazzy
- Pitch RTI (prti1516e.jar) at `{{pitch_rti_path}}`
- `turtlesim` package (for the Turtlesim example).
- `turtlebot4` package (for the TurtleBot4 example).
- `webots_ros2_turtlebot` package (for the Webots example).

## Configuration Guide

The bridge is driven by YAML configuration files found in `config/`.

### 1. HLA Settings
Defines the connection to the RTI.
```yaml
hla:
  federation_name: "RoboticsFed"
  federate_name: "ROS2Bridge" # Can use {{template}}
  fom_file_path: "/path/to/fom.xml"
  pitch_jar_path: "/path/to/prti1516e.jar"
```

### 2. Time Management
Controls how the federate advances time.
```yaml
time_management:
  is_regulating: true   # Drives the simulation time (Server)
  is_constrained: false # Obeys the simulation time (Client)
  time_step: 0.1        # Step size for time advance
  lookahead: 0.1        # Lookahead for regulating federates
```

### 3. ROS2 -> HLA (Publishing to HLA)
Maps a ROS2 topic to an HLA Object Attribute or Interaction.
```yaml
ros_to_hla:
  - ros_topic: "/turtle1/pose"
    ros_type: "turtlesim.msg.Pose"
    hla_object_class: "Robot"
    hla_instance_name: "Turtle1"
    mapping:
      x: "positionX"      # ROS field : HLA attribute
      y: "positionY"
      theta: "orientation"
```

### 4. HLA -> ROS2 (Subscribing from HLA)
Maps an HLA Object Attribute or Interaction to a ROS2 topic.
```yaml
hla_to_ros:
  - ros_topic: "/turtle1/cmd_vel"
    ros_type: "geometry_msgs.msg.Twist"
    hla_interaction_class: "Control"
    filter_parameter: "robotName" # Optional filtering
    filter_value: "Turtle1"
    mapping:
      linear.x: "linearVel"
      angular.z: "angularVel"
```

### 5. Templating
You can use `{{variable_name}}` in the YAML file. These variables are replaced at runtime by launch arguments.
Example: `ros_topic: "/{{robot_name}}/pose"` allows launching the same bridge for "Turtle1", "Turtle2", etc.

## Example 1: Turtlesim Collaborative Simulation
This example demonstrates a Server (Regulating) running the physics simulation and multiple Clients (Constrained) visualizing and controlling the robots.

### 1. Start the Server (Regulating Federate)
Launches the main simulation and the bridge.
```bash
ros2 launch ROS2HLA_Bridge server_turtlesim.launch.py
```

### 2. Start a Client (Constrained Federate)
Launches a client bridge and a local visualizer.
```bash
ros2 launch ROS2HLA_Bridge client_turtlesim.launch.py robot_name:=Turtle1
```

### 3. Control the Robot
The client exposes topics under the `/client/<robot_name>/` namespace.

To move `Turtle1`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Turtle1/cmd_vel
```

### Architecture (Turtlesim Example)

#### Server
- **Turtlesim Node**: The physics simulation.
- **Bridge Node**: Publishes `/turtleX/pose` -> HLA Object `Robot`. Subscribes HLA Interaction `Control` -> `/turtleX/cmd_vel`.

#### Client
- **Bridge Node**: Subscribes HLA Object `Robot` -> `/TurtleX/pose`. Publishes `/TurtleX/cmd_vel` -> HLA Interaction `Control`.
- **Turtlesim Node (Visualizer)**: A "ghost" simulation that only displays the turtle.
- **Visualizer Node**: Subscribes to `/TurtleX/pose` and calls `/TurtleX/teleport_absolute` to update the visualizer.

## Example 2: TurtleBot4 Gazebo Simulation
This example demonstrates controlling a TurtleBot4 in a Gazebo simulation via HLA.

### 1. Start the Server (Regulating Federate)
Launches the Gazebo simulation (Warehouse world) and the bridge.
```bash
ros2 launch ROS2HLA_Bridge server_turtlebot4.launch.py
```

### 2. Start the Client (Constrained Federate)
Launches the client bridge and an interactive teleop controller in the same terminal.
```bash
ros2 run ROS2HLA_Bridge client_interactive
```

### 3. Control the Robot
Use the keyboard keys (u, i, o, j, k, l, m, ,, .) in the client terminal to move the robot.
- **i**: Forward
- **k**: Stop
- **j**: Left
- **l**: Right
- **space**: Force Stop
- **d**: Undock
- **f**: Dock

## Example 3: Webots Simulation
This example demonstrates controlling a robot in a Webots simulation via HLA.

### 1. Prerequisites
Ensure you have the Webots ROS2 driver installed (e.g., `webots_ros2_turtlebot`).

### 2. Start Webots
Launch your Webots simulation.
```bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

### 3. Start the Server (Regulating Federate)
Launches the bridge configured for Webots.
```bash
ros2 launch ROS2HLA_Bridge server_webots.launch.py
```

### 4. Start the Client (Constrained Federate)
Launches the client bridge and an interactive teleop controller in the same terminal.
```bash
ros2 run ROS2HLA_Bridge client_interactive
```
