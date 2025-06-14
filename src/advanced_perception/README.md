# advanced_perception

Advanced perception package providing object segmentation and 6D pose estimation.

## Nodes
- `segmentation_node` – segments incoming images and publishes detected regions.
- `pose_estimation_node` – estimates 3D poses for segmented objects.

## Launch files
- `advanced_perception_launch.py` – starts both perception nodes with default configuration.

## Usage
Run the full stack:
```bash
ros2 launch advanced_perception advanced_perception_launch.py
```
Each node can also be started individually:
```bash
ros2 run advanced_perception segmentation_node
ros2 run advanced_perception pose_estimation_node
```

## Configuration
Configuration YAML files live under `config/`:
- `segmentation_config.yaml`
- `pose_estimation_config.yaml`
Pass the file path via the `segmentation_config` or `pose_estimation_config` parameters when launching if you need custom settings.
