# RealSense D435i Usage Guide

This guide explains how to use an Intel RealSense D435i camera with the Industrial Robotics Simulation Platform.

## 1. Install the ROS 2 Driver

Install the `realsense2_camera` package so ROS&nbsp;2 can access the device:

```bash
sudo apt install ros-$ROS_DISTRO-realsense2-camera
```

Confirm that the camera streams are available by running `ros2 topic list` after plugging in the device.

## 2. Configure the Platform

A sample configuration is provided at `src/simulation_core/config/realsense_config.yaml`.
It enables the D435i for perception and maps the default topics:

```yaml
camera_integration:
  type: realsense_d435i
  topic_mapping:
    color: /camera/color/image_raw
    depth: /camera/aligned_depth_to_color/image_raw
    pointcloud: /camera/depth/color/points
  camera_info_topic: /camera/color/camera_info
  frame_id: camera_color_optical_frame
```

Customize this file as needed and pass it to the launch files using the `config_dir` argument.

If the RealSense driver publishes `aligned_depth_to_color/image_raw` instead of
`/camera/depth/image_rect_raw`, remap the depth topic so that the perception
nodes receive the expected name. A minimal remapping file looks like:

```yaml
topic_mapping:
  "/camera/depth/image_rect_raw": "/camera/aligned_depth_to_color/image_raw"
```

You can pass this file to the launch system with the updated launch arguments:

```bash
ros2 launch simulation_tools integrated_system_launch.py \
  use_realsense:=true \
  config_dir:=src/simulation_core/config \
  --ros-args -r /camera/depth/image_rect_raw:=/camera/aligned_depth_to_color/image_raw
```

## 3. Launch with the Real Camera

Use the integrated launch file with the `use_realsense` flag:

```bash
ros2 launch simulation_tools integrated_system_launch.py \
  use_realsense:=true \
  config_dir:=src/simulation_core/config
```

Alternatively, run the helper script:

```bash
python scripts/run_sim.py --use-realsense
```

The launch file starts the `realsense2_camera_node` and connects it to the rest of the simulation. If the camera is not detected, the system falls back to the synthetic camera node.

## 4. Hybrid Mode

When combining real sensors with simulated robots, enable hybrid mode in the industrial protocol bridge:

```yaml
hybrid_mode: true
```

Refer to `industrial_deployment_guide.md` for details on mapping topics between real and simulated components.
