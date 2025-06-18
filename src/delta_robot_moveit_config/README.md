# delta_robot_moveit_config

## Purpose
MoveIt 2 configuration for controlling the delta robot.

## Setup
Build this package so the MoveIt configuration becomes available:

```bash
colcon build --packages-select delta_robot_moveit_config
source install/setup.bash
```

## Usage
Launch MoveIt together with RViz:

```bash
ros2 launch delta_robot_moveit_config move_group.launch.py
```

`move_group_headless.launch.py` starts MoveIt without a GUI.

## Extension
Configuration files are stored in the `config/` directory. Adapt them to change joint limits or planning parameters.
