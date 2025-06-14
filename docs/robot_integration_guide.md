# Robot Integration Guide

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
