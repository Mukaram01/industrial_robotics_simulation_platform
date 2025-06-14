# web_interface_backend

ROS 2 nodes implementing the Flask-based web dashboard.

## Nodes
- `web_interface_node` – serves the web UI and relays commands to the simulation.
- `visualization_server_node` – publishes composite images and saves data for export.

## Usage
These nodes are usually started via `simulation_tools/integrated_system_launch.py` but can be run directly:
```bash
ros2 run web_interface_backend web_interface_node
```
Adjust parameters such as `port`, `data_dir` or `jpeg_quality` when launching.
