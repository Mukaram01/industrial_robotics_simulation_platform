# perception_nodes

Collection of simple perception utilities including a synthetic camera simulator.

## Nodes
- `synthetic_camera_node` – publishes RGB and depth images for testing perception pipelines.

## Usage
Start the simulator:
```bash
ros2 run perception_nodes synthetic_camera_node
```
Parameters such as `frame_rate`, `resolution_width` and `object_count` can be set at launch time.
