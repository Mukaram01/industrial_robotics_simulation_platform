# Industrial Robotics Simulation Platform
# Comprehensive Deployment Guide

## Table of Contents
1. [Introduction](#introduction)
2. [System Overview](#system-overview)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [Operation](#operation)
6. [Industrial Use Cases](#industrial-use-cases)
7. [Hybrid Mode: Connecting to Real Equipment](#hybrid-mode-connecting-to-real-equipment)
8. [Advanced Features](#advanced-features)
9. [Customization](#customization)
10. [Troubleshooting](#troubleshooting)
11. [API Reference](#api-reference)
12. [Safety Considerations](#safety-considerations)
13. [Commercial Deployment](#commercial-deployment)

## Introduction

This guide provides comprehensive instructions for deploying, configuring, and operating the Industrial Robotics Simulation Platform. The platform is designed to simulate industrial robotics scenarios with high fidelity, supporting both demonstration purposes and integration with real hardware.

### Purpose

The Industrial Robotics Simulation Platform serves as:
- A demonstration tool for showcasing robotics capabilities to potential clients
- A development environment for testing and validating robotics applications
- A training platform for operators and engineers
- A foundation for a commercial robotics control system

### Key Features

- **Highly configurable simulation environment** with support for various industrial scenarios
- **Web-based GUI** for real-time monitoring and control
- **Dynamic object manipulation** allowing addition, removal, and modification of environment elements
- **Industrial protocol support** including OPC UA and MQTT
- **Safety monitoring system** with collision detection and emergency stop capabilities
- **Performance metrics tracking** for cycle time, throughput, and accuracy
- **Scenario recording and playback** for consistent demonstrations
- **Multi-view camera perspectives** for comprehensive visualization
- **Data export capabilities** for analysis and reporting
- **Hybrid mode** for connecting simulation to real equipment

## System Overview

The Industrial Robotics Simulation Platform consists of several integrated components:

1. **Camera Simulation**: Provides synthetic RGB and depth image streams simulating industrial cameras
2. **Environment Configuration**: Manages the simulation environment, objects, and physics
3. **Web Interface**: Provides a user-friendly interface for monitoring and control
4. **Visualization Server**: Generates visual representations of the simulation state
5. **Industrial Protocol Bridge**: Connects the simulation to industrial protocols
6. **Safety Monitor**: Enforces safety constraints and monitors for violations
7. **System Test**: Validates system functionality and integration

### Architecture Diagram

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Web Interface  │◄────┤  Environment    │◄────┤  Camera         │
│  (User Control) │     │  Configuration  │     │  Simulation     │
└────────┬────────┘     └────────┬────────┘     └─────────────────┘
         │                       │                       ▲
         │                       │                       │
         ▼                       ▼                       │
┌─────────────────┐     ┌─────────────────┐     ┌───────┴─────────┐
│  Visualization  │     │  Safety         │     │  Industrial     │
│  Server         │     │  Monitor        │     │  Protocol Bridge│
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

### Communication Flow

The system uses ROS2 topics for inter-component communication:

- `/camera/color/image_raw`: RGB camera images
- `/camera/depth/image_rect_raw`: Depth camera images
- `/robot/command`: Robot control commands
- `/robot/status`: Robot status information
- `/simulation/environment`: Environment state updates
- `/simulation/config`: Configuration updates
- `/simulation/metrics`: Performance metrics
- `/safety/status`: Safety monitoring status
- `/safety/emergency_stop`: Emergency stop signals
- `/visualization/combined_view`: Combined camera view
- `/visualization/metrics`: Metrics visualization

## Installation

### Prerequisites

- Ubuntu 22.04 or newer
- ROS2 Humble or newer
- Python 3.8 or newer
- Web browser (Chrome, Firefox, or Edge recommended)

### Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-yaml python3-matplotlib
sudo apt install -y ros-humble-cv-bridge ros-humble-sensor-msgs

# Install Python dependencies
pip3 install flask flask-socketio rclpy numpy opencv-python pyyaml matplotlib
pip3 install asyncua paho-mqtt  # For industrial protocol support
```

### UR5 Mesh Assets

The `ur5_robot_description` package provided in this repository installs a
`meshes` directory, but the actual UR5 mesh files are not included.
Before building the workspace either download the meshes from the
[Universal Robots description package](https://github.com/ros-industrial/universal_robot)
and place them in `src/ur5_robot_description/meshes`, or remove the `meshes`
entry from `src/ur5_robot_description/CMakeLists.txt`.

### Building the Workspace

1. Clone the repository:
```bash
git clone https://github.com/your-organization/industrial-robotics-simulation.git
cd industrial-robotics-simulation
```

2. Build the workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Configuration

### System Configuration

The system can be configured through YAML files located in the `config` directory:

- `default_camera_config.yaml`: Camera simulation parameters
- `default_sorting_config.yaml`: Object sorting parameters
- `safety_rules.yaml`: Safety monitoring rules
- `*.yaml`: Scenario configuration files

### Camera Configuration

Example camera configuration:

```yaml
simulation_mode: synthetic
frame_rate: 30.0
resolution:
  width: 640
  height: 480
camera_name: camera
object_count: 5
object_types:
  - red_cube
  - green_cylinder
  - blue_sphere
background_type: conveyor_belt
noise_level: 0.02
simulate_lighting: true
simulate_occlusion: false
```

### Environment Configuration

Example environment configuration:

```yaml
environment:
  type: factory
  size: [10.0, 10.0, 3.0]
  gravity: [0.0, 0.0, -9.81]
  ambient_light: 0.7
robots:
  - id: delta_robot_1
    type: delta_robot
    position: [0.0, 0.0, 1.5]
    orientation: [0.0, 0.0, 0.0, 1.0]
    parameters:
      speed: 1.0
      acceleration: 1.0
      workspace_radius: 0.5
      gripper_type: vacuum
conveyors:
  - id: conveyor_1
    type: belt
    position: [0.0, 0.0, 0.5]
    orientation: [0.0, 0.0, 0.0, 1.0]
    dimensions: [2.0, 0.5, 0.1]
    speed: 0.2
    direction: [0.0, 1.0, 0.0]
```

### Safety Configuration

Example safety configuration:

```yaml
collision_detection:
  enabled: true
  min_distance: 0.1
  objects: [robot, human, obstacle]
safety_zones:
  - name: robot_workspace
    type: cylinder
    center: [0.0, 0.0, 0.0]
    radius: 1.0
    height: 2.0
    restricted_objects: [human]
emergency_stop:
  auto_triggers: [collision, zone_violation, speed_violation]
  reset_requires_confirmation: true
```

### Industrial Protocol Configuration

Example industrial protocol configuration:

```yaml
opcua:
  enabled: true
  endpoint: opc.tcp://0.0.0.0:4840/freeopcua/server/
mqtt:
  enabled: true
  broker: localhost
  port: 1883
hybrid_mode: false
```

## Operation

### Starting the System

To start the complete system:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simulation_tools integrated_system_launch.py
```

To launch a specific scenario with advanced perception enabled:

```bash
ros2 launch simulation_tools integrated_system_launch.py \
    scenario:=warehouse use_advanced_perception:=true
```

### Accessing the Web Interface

Once the system is running, access the web interface at:

```
http://localhost:8080
```

### Basic Commands

The system accepts the following basic commands:

- `start`: Start the simulation
- `stop`: Stop the simulation
- `reset`: Reset the simulation to initial state
- `start_recording`: Start recording the simulation
- `stop_recording`: Stop recording and save
- `start_playback`: Play back the most recent recording
- `stop_playback`: Stop playback
- `emergency_stop`: Trigger emergency stop
- `reset_emergency_stop`: Reset emergency stop (if safe)

### Adding Objects

Objects can be added to the environment through the web interface or by publishing to the `/simulation/config` topic:

```json
{
  "add_object": {
    "id": "new_cube",
    "type": "cube",
    "position": [0.5, 0.5, 0.5],
    "dimensions": [0.1, 0.1, 0.1],
    "color": [1.0, 0.0, 0.0]
  }
}
```

### Modifying Objects

Objects can be modified through the web interface or by publishing to the `/simulation/config` topic:

```json
{
  "update_object": {
    "id": "existing_cube",
    "position": [1.0, 1.0, 0.5],
    "color": [0.0, 1.0, 0.0]
  }
}
```

### Removing Objects

Objects can be removed through the web interface or by publishing to the `/simulation/config` topic:

```json
{
  "remove_object": "object_id"
}
```

## Industrial Use Cases

The platform supports various industrial use cases, including:

### Robot Arm Pick and Place

A robot arm picks items from a basket and places them in an organized manner on a conveyor belt.

Configuration example:

```yaml
robots:
  - id: robot_arm
    type: ur5
    position: [0.0, 0.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    parameters:
      speed: 1.0
      acceleration: 0.5
      gripper_type: parallel
containers:
  - id: basket
    type: basket
    position: [-0.5, 0.0, 0.1]
    dimensions: [0.3, 0.3, 0.2]
conveyors:
  - id: output_conveyor
    type: belt
    position: [0.5, 0.0, 0.1]
    dimensions: [1.0, 0.4, 0.1]
    speed: 0.1
    direction: [0.0, 1.0, 0.0]
```

### Delta Robot Trash Sorting

A delta robot sorts different types of trash from a conveyor belt into separate bins.

Configuration example:

```yaml
robots:
  - id: delta_robot
    type: delta
    position: [0.0, 0.0, 1.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    parameters:
      speed: 2.0
      acceleration: 5.0
      workspace_radius: 0.6
      gripper_type: vacuum
conveyors:
  - id: input_conveyor
    type: belt
    position: [0.0, -0.5, 0.1]
    dimensions: [0.5, 2.0, 0.1]
    speed: 0.2
    direction: [0.0, 1.0, 0.0]
containers:
  - id: bin_plastic
    type: bin
    position: [-0.5, 0.5, 0.0]
    dimensions: [0.3, 0.3, 0.3]
    color: [0.0, 0.0, 1.0]
  - id: bin_metal
    type: bin
    position: [0.0, 0.5, 0.0]
    dimensions: [0.3, 0.3, 0.3]
    color: [1.0, 0.0, 0.0]
  - id: bin_paper
    type: bin
    position: [0.5, 0.5, 0.0]
    dimensions: [0.3, 0.3, 0.3]
    color: [0.0, 1.0, 0.0]
```

### Quality Control Inspection

A robot arm with a camera inspects products on a conveyor belt for defects.

Configuration example:

```yaml
robots:
  - id: inspection_robot
    type: ur5
    position: [0.0, 0.0, 0.5]
    orientation: [0.0, 0.0, 0.0, 1.0]
    parameters:
      speed: 0.5
      acceleration: 0.2
      camera_attached: true
conveyors:
  - id: inspection_conveyor
    type: belt
    position: [0.0, 0.0, 0.1]
    dimensions: [0.5, 2.0, 0.1]
    speed: 0.1
    direction: [0.0, 1.0, 0.0]
containers:
  - id: pass_bin
    type: bin
    position: [0.5, 0.5, 0.0]
    dimensions: [0.3, 0.3, 0.3]
    color: [0.0, 1.0, 0.0]
  - id: fail_bin
    type: bin
    position: [-0.5, 0.5, 0.0]
    dimensions: [0.3, 0.3, 0.3]
    color: [1.0, 0.0, 0.0]
```

## Hybrid Mode: Connecting to Real Equipment

The platform supports hybrid mode, where the simulation can be connected to real equipment.

### Enabling Hybrid Mode

To enable hybrid mode, set the `hybrid_mode` parameter to `true` in the industrial protocol bridge configuration:

```yaml
hybrid_mode: true
```

### Hardware Requirements

- Industrial robot with ROS2 driver or OPC UA/MQTT interface
- Camera with ROS2 driver (e.g., Intel RealSense D435i)
- Industrial PLC with OPC UA/MQTT support (optional)

### Connecting to a Real Robot

1. Configure the robot's IP address in the industrial protocol bridge:

```yaml
robot_connection:
  type: opcua  # or mqtt, ros2
  address: 192.168.1.100
  port: 4840
```

2. Map simulation topics to real robot topics:

```yaml
topic_mapping:
  "/robot/command": "/real_robot/command"
  "/robot/status": "/real_robot/status"
```

### Connecting to a Real Camera

1. Configure the camera in the launch file:

```python
# Replace camera simulator with real camera node
real_camera_node = LaunchNode(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    name='camera',
    parameters=[{
        'enable_color': True,
        'enable_depth': True,
        'color_width': 640,
        'color_height': 480,
        'depth_width': 640,
        'depth_height': 480
    }]
)
```

2. Update topic mappings if necessary:

```yaml
topic_mapping:
  "/camera/color/image_raw": "/camera/color/image_raw"
  "/camera/depth/image_rect_raw": "/camera/aligned_depth_to_color/image_raw"
```

## Advanced Features

### Performance Metrics Dashboard

The system tracks and visualizes key performance metrics:

- **Cycle Time**: Time to complete one operation cycle
- **Throughput**: Number of objects processed per minute
- **Accuracy**: Percentage of correctly processed objects
- **Errors**: Count of errors encountered

Access the metrics dashboard at:

```
http://localhost:8080/dashboard
```

### Scenario Recording and Playback

Record and play back scenarios for consistent demonstrations:

1. Start recording: `ros2 topic pub /robot/command std_msgs/String "data: 'start_recording'"`
2. Perform operations
3. Stop recording: `ros2 topic pub /robot/command std_msgs/String "data: 'stop_recording'"`
4. Play back: `ros2 topic pub /robot/command std_msgs/String "data: 'start_playback'"`

Recordings are saved in the `data/recordings` directory.

### Error Simulation

Simulate various error conditions to test system robustness:

```json
{
  "action": "trigger_error",
  "type": "gripper_failure"
}
```

Supported error types:
- `gripper_failure`: Gripper fails to grasp object
- `object_slip`: Object slips from gripper
- `sensor_noise`: Increased sensor noise
- `communication_delay`: Delayed communication
- `power_fluctuation`: Power supply issues

### Multi-View Camera Perspectives

Switch between different camera views:

```json
{
  "action": "set_camera",
  "id": "camera_side"
}
```

### Data Export

Export simulation data for analysis:

1. Enable export: Set `export_enabled: true` in visualization server configuration
2. Configure export interval: Set `export_interval: 60.0` (seconds)

Exported data is saved in the `data/exports` directory.

## Customization

### Adding New Object Types

1. Define the object in the environment configuration:

```yaml
object_templates:
  - id: custom_object
    type: custom
    dimensions: [0.1, 0.1, 0.2]
    color: [0.5, 0.5, 0.0]
    mass: 0.2
    physics_properties:
      friction: 0.7
      restitution: 0.3
```

2. Implement rendering in the camera simulator:

```python
def render_custom_object(self, obj, rgb_image, depth_image):
    # Custom rendering code
    pass
```

### Adding New Robot Types

1. Define the robot in the environment configuration:

```yaml
robot_templates:
  - id: custom_robot
    type: custom
    joints: 6
    workspace_radius: 0.8
    max_payload: 5.0
    kinematics_file: "custom_robot_kinematics.yaml"
```

2. Implement kinematics in a separate file:

```yaml
# custom_robot_kinematics.yaml
dh_parameters:
  - [0.0, 1.57, 0.1625, 0.0]
  - [0.0, 0.0, 0.425, 0.0]
  - [0.0, 0.0, 0.3922, 0.0]
  - [0.0, 1.57, 0.0, 0.0]
  - [0.0, -1.57, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0]
```

### Implementing Custom Algorithms

1. Create a new ROS2 node:

```python
class CustomAlgorithmNode(Node):
    def __init__(self):
        super().__init__('custom_algorithm_node')
        # Implementation
```

2. Add the node to the launch file:

```python
custom_algorithm_node = LaunchNode(
    package='simulation_tools',
    executable='custom_algorithm_node',
    name='custom_algorithm_node',
    parameters=[{
        'param1': value1,
        'param2': value2
    }]
)
ld.add_action(custom_algorithm_node)
```

### Integrating Pre-trained Models

1. Place model files in the `models` directory:

```
models/
  object_detection/
    model.onnx
    labels.txt
  pose_estimation/
    model.onnx
    config.json
```

2. Configure model loading in the appropriate node:

```python
def load_model(self):
    model_path = os.path.join(
        get_package_share_directory('simulation_tools'),
        'models',
        'object_detection',
        'model.onnx'
    )
    self.model = cv2.dnn.readNetFromONNX(model_path)
```

## Troubleshooting

### Common Issues

#### Web Interface Not Accessible

**Symptoms**: Cannot access the web interface at http://localhost:8080

**Solutions**:
- Check if the web interface node is running: `ros2 node list | grep web_interface`
- Verify the port is not in use: `sudo netstat -tuln | grep 8080`
- Check for firewall issues: `sudo ufw status`

#### Camera Images Not Appearing

**Symptoms**: No camera images in the web interface

**Solutions**:
- Check if the camera simulator is running: `ros2 node list | grep camera_simulator`
- Verify topics are being published: `ros2 topic echo /camera/color/image_raw`
- Check for errors in the camera simulator logs: `ros2 topic echo /rosout | grep camera_simulator`

#### Physics Simulation Issues

**Symptoms**: Objects behave unrealistically or pass through each other

**Solutions**:
- Check physics parameters in the environment configuration
- Reduce simulation speed: Set `physics_timestep` to a smaller value
- Verify object collision geometries

#### Industrial Protocol Connection Failures

**Symptoms**: Cannot connect to OPC UA or MQTT

**Solutions**:
- Check if the required packages are installed: `pip list | grep asyncua`
- Verify broker/server is running: `telnet localhost 1883` (for MQTT)
- Check network connectivity: `ping <broker_address>`

### Diagnostic Tools

#### System Status Check

```bash
ros2 topic pub /test/command std_msgs/String "data: 'run_diagnostics'"
```

#### Topic Monitoring

```bash
ros2 topic list
ros2 topic echo /simulation/environment
```

#### Node Information

```bash
ros2 node list
ros2 node info /environment_configurator_node
```

#### Parameter Inspection

```bash
ros2 param list
ros2 param get /camera_simulator_node simulation_mode
```

## API Reference

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera image |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth camera image |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration info |
| `/robot/command` | `std_msgs/String` | Robot control commands |
| `/robot/status` | `std_msgs/String` | Robot status information |
| `/simulation/environment` | `std_msgs/String` | Environment state (JSON) |
| `/simulation/config` | `std_msgs/String` | Configuration updates (JSON) |
| `/simulation/metrics` | `std_msgs/String` | Performance metrics (JSON) |
| `/safety/status` | `std_msgs/String` | Safety monitoring status (JSON) |
| `/safety/emergency_stop` | `std_msgs/Bool` | Emergency stop signal |
| `/visualization/combined_view` | `sensor_msgs/Image` | Combined camera view |
| `/visualization/metrics` | `sensor_msgs/Image` | Metrics visualization |

### Web API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web interface home page |
| `/dashboard` | GET | Metrics dashboard |
| `/config` | GET | Configuration interface |
| `/scenarios` | GET | Scenario management |
| `/api/scenarios` | GET | List available scenarios |
| `/api/scenarios` | POST | Save a new scenario |
| `/api/scenarios/<name>` | GET | Get scenario details |
| `/api/scenarios/<name>` | DELETE | Delete a scenario |
| `/api/status` | GET | Get system status |
| `/api/image/latest` | GET | Get latest camera image |

### Socket.IO Events

| Event | Direction | Description |
|-------|-----------|-------------|
| `connect` | Client → Server | Client connection |
| `disconnect` | Client → Server | Client disconnection |
| `start_simulation` | Client → Server | Start simulation |
| `stop_simulation` | Client → Server | Stop simulation |
| `send_command` | Client → Server | Send robot command |
| `update_config` | Client → Server | Update configuration |
| `add_object` | Client → Server | Add object to environment |
| `remove_object` | Client → Server | Remove object from environment |
| `modify_object` | Client → Server | Modify object properties |
| `status_update` | Server → Client | System status update |
| `image_update` | Server → Client | New camera image available |
| `depth_update` | Server → Client | New depth image available |
| `command_sent` | Server → Client | Command sent confirmation |
| `config_updated` | Server → Client | Configuration updated |
| `object_added` | Server → Client | Object added confirmation |
| `object_removed` | Server → Client | Object removed confirmation |
| `object_modified` | Server → Client | Object modified confirmation |

## Safety Considerations

### Virtual Safety Features

The simulation includes several safety features:

1. **Collision Detection**: Monitors for potential collisions between objects
2. **Safety Zones**: Defines restricted areas for different object types
3. **Speed Limits**: Enforces maximum speed for robots and conveyors
4. **Emergency Stop**: Provides immediate system halt capability
5. **Safety Monitoring**: Continuously checks for safety violations

### Real-World Safety Integration

When connecting to real equipment:

1. **Hardware Emergency Stop**: Connect physical e-stop buttons to the system
2. **Safety PLC Integration**: Interface with safety PLCs via industrial protocols
3. **Compliance Monitoring**: Ensure operations comply with safety standards
4. **Operator Presence Detection**: Integrate sensors for human detection
5. **Safety Certification**: Follow relevant safety standards (ISO/TS 15066, ISO 13849)

### Safety Standards Compliance

The system is designed with the following safety standards in mind:

- ISO/TS 15066: Robots and robotic devices — Collaborative robots
- ISO 13849: Safety of machinery — Safety-related parts of control systems
- IEC 61508: Functional Safety of Electrical/Electronic/Programmable Electronic Safety-related Systems

## Commercial Deployment

### Licensing

The Industrial Robotics Simulation Platform is provided under a commercial license. Contact sales@your-company.com for licensing information.

### Support and Maintenance

Support options:
- Standard support: Business hours email support
- Premium support: 24/7 phone and email support
- Custom support: Tailored to your organization's needs

### Training

Training options:
- Online self-paced training
- Instructor-led virtual training
- On-site training and workshops

### Customization Services

- Custom scenario development
- Integration with existing systems
- Custom robot and equipment models
- Specialized algorithm development
- Hardware integration services

### Deployment Options

1. **On-premises**: Deploy on your own hardware
2. **Cloud-based**: Deploy on cloud infrastructure
3. **Hybrid**: Combination of on-premises and cloud components

### Hardware Requirements

Minimum requirements:
- CPU: 4 cores, 2.5 GHz
- RAM: 8 GB
- GPU: NVIDIA GTX 1060 or equivalent
- Storage: 20 GB SSD

Recommended requirements:
- CPU: 8+ cores, 3.5 GHz
- RAM: 16+ GB
- GPU: NVIDIA RTX 2070 or better
- Storage: 100+ GB SSD

### Scaling Considerations

For large-scale deployments:
- Distributed simulation across multiple nodes
- Load balancing for web interface
- Database integration for metrics and configuration
- High-availability setup for critical applications
