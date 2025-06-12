# Full System Step-by-Step Guide

This guide walks through launching every component of the simulation platform: Gazebo, the UR5 robot, object detection, RViz, and the web interface.

## Prerequisites

- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- The workspace built as described in the main README
- Gazebo packages installed (`sudo apt install ros-humble-gazebo-ros-pkgs`)

## 1. Build the Workspace

```bash
cd industrial_robotics_simulation_platform
pip install -r requirements.txt
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## 2. Start the Integrated Simulation

Open a terminal and launch the main system (camera simulator, protocol bridge, safety monitor and web server):

```bash
ros2 launch simulation_tools integrated_system_launch.py \
    use_realsense:=false use_advanced_perception:=true
```

To begin publishing camera data and metrics, send a start command:

```bash
ros2 topic pub /simulation/command std_msgs/msg/String '{data: "start"}'
```

Using the "Start" button on the web interface performs the same action.

This also starts the Flask-based web interface on port `8080`.

## 3. Launch Object Detection

In a second terminal run the object detection pipeline:

```bash
ros2 launch apm_core object_detection_launch.py
```

## 4. Visualize with RViz

Open another terminal for RViz to view the UR5 model and camera feeds:

```bash
ros2 launch ur5_robot_description display.launch.py
```

## 5. (Optional) Run Gazebo

To see the robot in a physics simulation, start Gazebo in a separate terminal:

```bash
ros2 launch gazebo_ros empty_world.launch.py
```

Then spawn the UR5 into Gazebo using the description published by `robot_state_publisher`:

```bash
ros2 run gazebo_ros spawn_entity.py \
    -topic /robot_description -entity ur5
```

## 6. Access the Web Interface

Navigate to [http://localhost:8080](http://localhost:8080) in your browser to control the simulation, add objects and monitor status.

The full system is now running with Gazebo, RViz, object detection and the web dashboard.
