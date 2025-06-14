# simulation_tools

Utility launch files that combine the various components of the platform.

## Launch files
- `integrated_system_launch.py` – brings up the camera, environment configurator, web interface and protocol bridge.
- `realsense_hybrid_launch.py` – variant using an Intel RealSense camera.
- `visualization_launch.py` – standalone visualization server.

## Usage
Start the full system:
```bash
ros2 launch simulation_tools integrated_system_launch.py
```
See available launch arguments by passing `-h`.
