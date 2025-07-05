# simulation_tools

## Purpose
Utility launch files that combine the various components of the platform.

## Setup
Install the package so the launch files are discoverable:

```bash
colcon build --packages-select simulation_tools
source install/setup.bash
```

## Usage
Start the full system:

```bash
ros2 launch simulation_tools integrated_system_launch.py
```

Other launches include `realsense_hybrid_launch.py` for RealSense cameras and `visualization_launch.py` for a standalone visualization server.

## Extension
Pass `-h` to any launch file to see configurable arguments and adapt them to your needs.

Both `integrated_system_launch.py` and `realsense_hybrid_launch.py` accept a
`camera_config` argument that points to a YAML file with parameters for the
`realsense2_camera` node. The default is the sample configuration in
`simulation_core/config/realsense_config.yaml`.
