# Universal ROS2-HLA Bridge

This project implements a **Universal Bridge** that connects **ROS2** (Robot Operating System 2) with **HLA** (High Level Architecture) simulations. It allows for bidirectional communication and time synchronization between the two domains.

## Features

*   **Universal & Configurable**: No hardcoded mappings. Everything is defined in `bridge_config.yaml`.
*   **Automatic FOM Generation**: The HLA Object Model (FOM) is generated automatically from the configuration.
*   **Dynamic Types**: Supports any standard ROS2 message type without recompilation.
*   **Bidirectional Communication**:
    *   **ROS2 -> HLA**: Subscribes to ROS2 topics and updates HLA Object Attributes.
    *   **HLA -> ROS2**: Reflects HLA Attribute updates and publishes to ROS2 topics.
*   **Time Synchronization**:
    *   **HLA Master**: HLA drives the simulation time; ROS2 follows via `/clock`.
    *   **ROS2 Master**: ROS2 (e.g., Gazebo) drives the time; HLA follows via Time Advance Requests.

## Architecture

```mermaid
graph TD
    subgraph ROS2 Domain
        Node[ROS2 Node]
        TopicPub[Topic Publisher]
        TopicSub[Topic Subscriber]
        Clock[/clock]
    end

    subgraph Bridge
        Config[bridge_config.yaml]
        FOMGen[fom_generator.py]
        BridgeApp[universal_bridge.py]
    end

    subgraph HLA Domain
        RTI[RTI (Pitch/Mak)]
        Federate[Other Federate]
    end

    Config --> FOMGen
    FOMGen -->|Generates| XML[universal_fom.xml]
    Config --> BridgeApp
    XML --> BridgeApp

    Node -->|Publish| TopicPub
    TopicPub -->|Subscribe| BridgeApp
    BridgeApp -->|UpdateAttribute| RTI

    RTI -->|ReflectAttribute| BridgeApp
    BridgeApp -->|Publish| TopicSub
    TopicSub -->|Subscribe| Node

    BridgeApp <-->|Time Management| RTI
    BridgeApp <-->|/clock| Clock
```

## Installation

1.  **Prerequisites**:
    *   ROS2 (Jazzy/Humble)
    *   Python 3
    *   HLA RTI (e.g., Pitch pRTI Free)
    *   `jpype1`
    *   `pyyaml`

2.  **Setup**:
    *   Ensure `PITCH_JAR` in `universal_bridge.py` points to your `prti1516e.jar`.
    *   Source your ROS2 environment: `source /opt/ros/jazzy/setup.bash`.

## Configuration (`bridge_config.yaml`)

```yaml
federation_name: "RosHlaFed"
fom_name: "UniversalRosFom"

# Time Synchronization Mode
time:
  mode: "hla_master" # "hla_master", "ros_master", or "none"
  step: 0.1          # Step size in seconds
  lookahead: 0.1     # Lookahead for regulation

nodes:
  turtlesim:
    object_class: "Turtlesim"
    attributes:
      pose:
        ros_topic: "/turtle1/pose"
        ros_type: "turtlesim.msg.Pose"
        direction: "ros2_to_hla" # Bridge sends to HLA
      cmd_vel:
        ros_topic: "/turtle1/cmd_vel"
        ros_type: "geometry_msgs.msg.Twist"
        direction: "hla_to_ros2" # Bridge receives from HLA
```

## Usage

1.  **Start the Bridge**:
    ```bash
    cd src
    python3 universal_bridge.py
    ```
    This will generate the FOM in `../generated/`, join the federation, and start the bridge.

2.  **Verify**:
    *   **ROS2 Side**: Run `python3 tests/mock_turtlesim.py` (or real turtlesim).
    *   **HLA Side**: Run `python3 tests/hla_controller.py`.
    *   **Time**: Run `python3 tests/time_verifier.py` to check clock sync.

## Ownership Model

*   **ROS2 -> HLA**: The Bridge registers the Object Instance and retains ownership of these attributes.
*   **HLA -> ROS2**: The Bridge registers the Object Instance but **divests** ownership of these attributes immediately. This allows other federates (like a Controller) to acquire ownership and send updates.
