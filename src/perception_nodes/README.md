# perception_nodes

## Purpose
Collection of simple perception utilities including a synthetic camera simulator.

## Setup
Build this package and source the environment:

```bash
colcon build --packages-select perception_nodes
source install/setup.bash
```

## Usage
Start the simulator:

```bash
ros2 run perception_nodes synthetic_camera_node
```

## Extension
Parameters such as `frame_rate`, `resolution_width` and `object_count` can be adjusted in the launch file or via command line arguments.
