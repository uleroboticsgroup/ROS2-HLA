# ROS2HLA_Bridge

This package bridges ROS2 and HLA (IEEE 1516e) for collaborative simulation.

## Prerequisites
- ROS2 Jazzy
- Pitch RTI (prti1516e.jar) at `/home/vicen/prti1516e/lib/prti1516e.jar`

## Structure
- `config/`: YAML configuration files defining the mapping between ROS2 topics and HLA Objects/Interactions.
- `fom/`: HLA Federation Object Model (FOM) files.
- `launch/`: Launch files.
- `ROS2HLA_Bridge/`: Python source code.

## Usage

### 1. Start the Simulation (Server/Bridge)
This launches Turtlesim and the Bridge configured to publish Turtle1's pose and subscribe to commands.

```bash
ros2 launch ROS2HLA_Bridge bridge.launch.py
```

### 2. Start a Client
This launches a bridge acting as a client. It subscribes to Turtle1's pose and publishes commands.
You can run multiple clients with different robot names (if supported by the sim).

```bash
ros2 launch ROS2HLA_Bridge client.launch.py robot_name:=Turtle1
```

To control the robot from the client side, you can publish to the client's command topic (which is bridged to HLA):

```bash
ros2 topic pub /Turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

Or run teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/Turtle1/cmd_vel
```

## Configuration
Edit `config/bridge_config.yaml` to add more robots or topics.
The bridge supports `{{robot_name}}` templating for scalable client configurations.
