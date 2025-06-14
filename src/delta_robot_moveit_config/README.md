# delta_robot_moveit_config

MoveIt 2 configuration for controlling the delta robot.

## Launch files
- `move_group.launch.py` – start MoveIt with RViz for interactive motion planning.
- `move_group_headless.launch.py` – MoveIt without a GUI.

## Usage
Launch MoveIt with RViz:
```bash
ros2 launch delta_robot_moveit_config move_group.launch.py
```
