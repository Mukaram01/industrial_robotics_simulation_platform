# ur5_robot_moveit_config

## Purpose
MoveIt 2 configuration for the UR5 manipulator.

## Setup
Build this package so the MoveIt configuration becomes available:

```bash
colcon build --packages-select ur5_robot_moveit_config
source install/setup.bash
```

## Usage
```bash
ros2 launch ur5_robot_moveit_config move_group.launch.py
```

## Extension
Requires the UR5 description from `ur5_robot_description`. Configuration files reside in `config/` and can be customised for different planners or kinematics settings.
