# delta_robot_description

## Purpose
URDF and mesh resources describing a delta robot.

## Setup
Build the description package so it can be found by other packages:

```bash
colcon build --packages-select delta_robot_description
source install/setup.bash
```

## Usage
Visualize the robot in RViz:

```bash
ros2 launch delta_robot_description display.launch.py
```

`display_headless.launch.py` starts only the state publisher.

## Extension
URDF files reside in the `urdf/` directory and meshes under `meshes/`. Modify these resources to adapt the robot model.
