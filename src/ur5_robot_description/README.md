# ur5_robot_description

## Purpose
UR5 robot model files used for visualization and MoveIt.

## Setup
Build the package so that the description can be located:

```bash
colcon build --packages-select ur5_robot_description
source install/setup.bash
```

## Usage
Launch the UR5 model in RViz:

```bash
ros2 launch ur5_robot_description display.launch.py
```

## Extension
Mesh resources should either come from the `ur_description` package or be placed in the local `meshes/` directory. Modify the URDF under `urdf/` to adapt the robot model.
