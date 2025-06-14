# Robot Integration Guide

This guide outlines the basic steps for adding a new robot to the simulation platform.

## 1. Create a Description Package

1. Start by creating a new package for your robot's URDF/Xacro files.
2. Copy the templates located in `templates/robot_description` to your package:

```bash
cp templates/robot_description/template_robot.urdf.xacro <your_pkg>/urdf/
cp templates/robot_description/template_robot.srdf <your_pkg>/config/
```

3. Edit the copied files and replace the placeholder comments with your robot's link and joint definitions.

## 2. Update Launch and Configuration Files

After defining the robot, update your launch files and MoveIt configuration to reference the new description files.

Refer to the existing robot packages in `src/` for examples of how everything fits together.

This guide describes how to connect a new robot implementation to the simulation platform. All robots are expected to communicate with the rest of the system using a set of common ROS2 topics and, where appropriate, services or action interfaces. Following these conventions allows different robot drivers to be swapped without modifying other packages.

## Common ROS2 interfaces

| Interface | Type | Description |
|-----------|------|-------------|
| `/robot/command` | `std_msgs/String` | Robot motion or mode commands. Examples include `start`, `stop`, `pick`, and `place`. |
| `/robot/status` | `std_msgs/String` or `apm_msgs/RobotStatus` | High level status such as `idle`, `running`, or error information. |
| `/robot/joint_states` | `sensor_msgs/JointState` | Current joint positions reported by the robot. |
| `/simulation/command` | `std_msgs/String` | Simulation control commands that mirror `/robot/command`. |
| `/simulation/status` | `std_msgs/String` | Simulation status messages that mirror `/robot/status`. |

Nodes provided in this repository publish and subscribe to the `/simulation/*` topics by default. When integrating a real robot, remap these to the corresponding `/robot/*` interfaces so all other components continue to operate without changes.
