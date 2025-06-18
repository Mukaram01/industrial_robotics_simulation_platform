# advanced_perception

## Purpose
Advanced perception package providing object segmentation and 6D pose estimation.

## Setup
Build the workspace and source the environment:

```bash
colcon build --packages-select advanced_perception
source install/setup.bash
```

## Usage
Launch both perception nodes with the default configuration:

```bash
ros2 launch advanced_perception advanced_perception_launch.py
```

You can also run the nodes individually:

```bash
ros2 run advanced_perception segmentation_node
ros2 run advanced_perception pose_estimation_node
```

## Extension
YAML files under `config/` provide parameters for each node:
- `segmentation_config.yaml`
- `pose_estimation_config.yaml`

Pass these paths through the `segmentation_config` or `pose_estimation_config` parameters when launching to override the defaults.
