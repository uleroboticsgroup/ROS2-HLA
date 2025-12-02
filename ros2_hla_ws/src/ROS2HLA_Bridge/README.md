# ROS2HLA_Bridge

This package provides a generic bridge between **ROS2** (Robot Operating System 2) and **HLA** (High Level Architecture, IEEE 1516e) to enable collaborative simulation across different environments.

While the core bridge is agnostic and configurable for any ROS2 system, this package includes a fully functional example using **Turtlesim** to demonstrate its capabilities.

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

## Architecture (Turtlesim Example)

### Server
- **Turtlesim Node**: The physics simulation.
- **Bridge Node**: Publishes `/turtleX/pose` -> HLA Object `Robot`. Subscribes HLA Interaction `Control` -> `/turtleX/cmd_vel`.

### Client
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

The client terminal will also display the robot's odometry received from the simulation.
