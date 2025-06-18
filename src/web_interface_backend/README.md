# web_interface_backend

## Purpose
ROS 2 nodes implementing the Flask-based web dashboard.

## Setup
Build the package and source the environment:

```bash
colcon build --packages-select web_interface_backend
source install/setup.bash
```

## Usage
These nodes are usually started via `simulation_tools/integrated_system_launch.py` but can be run directly:

```bash
ros2 run web_interface_backend web_interface_node
```

## Extension
Adjust parameters such as `port`, `data_dir` or `jpeg_quality` when launching. Configuration files reside under `resource/`.
