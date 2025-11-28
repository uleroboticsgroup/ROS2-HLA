# TurtleBot4 HLA Demo

This demo shows how to control a simulated TurtleBot4 in Gazebo from an HLA Federation.

## Prerequisites
- `turtlebot4_simulator` installed (`sudo apt install ros-jazzy-turtlebot4-simulator` or similar).
- `ros-jazzy-turtlebot4-desktop`

## Architecture
- **Gazebo**: Runs the physics simulation and ROS2 nodes. Acts as the **Time Master**.
- **Bridge**: Connects to ROS2, subscribes to `/clock` (to sync time), sends `/odom` to HLA, and receives `/cmd_vel` from HLA.
- **HLA Controller**: A simple python script (or any other federate) that sends velocity commands.

## Steps

### 1. Launch Simulation
Launch the TurtleBot4 simulation in Gazebo.
```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
```
*Note: Ensure the simulation is running and you can see the robot.*

### 2. Run the Bridge
Run the bridge with the TurtleBot4 configuration.
```bash
cd src
python3 universal_bridge.py ../config/turtlebot4_config.yaml
```

### 3. Run the Undock Adapter
In a new terminal, run the adapter to handle the undock action.
```bash
python3 src/undock_adapter.py
```

### 4. Run the Controller
Run the HLA controller to trigger undock and drive the robot.
```bash
python3 tests/turtlebot4_controller.py
```
*The controller will trigger undock, wait 20s, and then start driving in circles.*

## Data Flow
1.  **Odometry**: Gazebo -> `/odom` -> Bridge -> HLA `TurtleBot.pose` (JSON)
2.  **Commands**: HLA `TurtleBot.cmd_vel` (JSON) -> Bridge -> `/cmd_vel` -> Gazebo

## Troubleshooting
- **Time not advancing**: Ensure Gazebo is not paused.
- **Bridge stuck**: Check if `ros_master` mode is enabled and `/clock` is being published.
