# apm_core

## Purpose
Advanced Perception Module containing object detection and helper nodes.

## Setup
Build this package and source the workspace:

```bash
colcon build --packages-select apm_core
source install/setup.bash
```

## Usage
Launch the object detection pipeline:

```bash
ros2 launch apm_core object_detection_launch.py
```

Individual nodes can be started manually:

```bash
ros2 run apm_core onnx_inference_node
```

## Extension
`config/default_object_detection_config.yaml` contains paths and parameters for the ONNX model. Class names are stored in `config/coco_classes.txt`. The `models/` directory holds the example model files and can be replaced with custom networks.
