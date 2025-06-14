# apm_core

Advanced Perception Module containing object detection and helper nodes.

## Nodes
- `onnx_inference_node` – runs an ONNX object detection network and publishes detections.
- `image_subscriber_node` – simple camera subscriber example.
- `point_cloud_subscriber_node` – receives depth point clouds.

## Launch files
- `object_detection_launch.py` – starts the inference node with the default model configuration.
- `apm_subscribers_launch.py` – launches example image and point cloud subscribers.

## Usage
Launch the detection pipeline:
```bash
ros2 launch apm_core object_detection_launch.py
```
Or run an individual node:
```bash
ros2 run apm_core onnx_inference_node
```

## Configuration
`config/default_object_detection_config.yaml` contains paths and parameters for the ONNX model. Class names are stored in `config/coco_classes.txt`. The `models/` directory holds the example model files.
