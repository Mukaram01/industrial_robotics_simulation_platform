# Full System Step-by-Step Guide

This guide walks through launching every component of the simulation platform: Gazebo, the UR5 robot, object detection, RViz, and the web interface.

## Prerequisites

- Ubuntu 22.04 or newer with [ROS&nbsp;2 Humble](https://docs.ros.org/en/humble/Installation.html) installed (`source /opt/ros/humble/setup.bash`)
- Gazebo packages installed (`sudo apt install ros-humble-gazebo-ros-pkgs`)
- `python3-rosdep` installed and initialized
- Docker can be used to run the platform if you prefer not to install ROS&nbsp;2 locally

## 1. Build the Workspace

```bash
cd industrial_robotics_simulation_platform
pip install -r requirements.txt
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## 2. Start the Integrated Simulation

Open a terminal and launch the main system (camera simulator, protocol bridge, safety monitor and web server):

```bash
ros2 launch simulation_core full_system.launch.py \
    use_realsense:=false use_advanced_perception:=true
```

If you prefer to start components separately for debugging, launch them
individually:

```bash
ros2 launch simulation_core physics_server.launch.py
ros2 launch simulation_core environment_manager.launch.py
ros2 launch simulation_core web_interface.launch.py
ros2 launch simulation_core visualization_server.launch.py
ros2 launch simulation_core industrial_bridge.launch.py
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

## 7. Adjust Error Simulation Rate

The environment configurator node can randomly trigger errors during metrics
updates. Control how often this happens using the `error_simulation_rate`
parameter. A value of `0.0` disables error injection while `1.0` forces an error
every cycle.

Set the rate when launching the system:

```bash
ros2 launch simulation_core full_system.launch.py error_simulation_rate:=0.2
```

You can also change the value at runtime by publishing a configuration message:

```bash
ros2 topic pub /simulation/config std_msgs/msg/String '{"settings": {"simulation": {"error_simulation_rate": 0.5}}}'
```
