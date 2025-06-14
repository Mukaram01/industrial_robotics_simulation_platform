# ur5_robot_description

UR5 robot model files used for visualization and MoveIt.

## Launch files
- `display.launch.py` – start `robot_state_publisher` and RViz for the UR5 model.

## Usage
```bash
ros2 launch ur5_robot_description display.launch.py
```
Mesh resources must be available from the `ur_description` package or placed in the `meshes/` directory.
