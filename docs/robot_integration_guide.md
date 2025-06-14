# Robot Integration Guide

This guide explains how to integrate a new robot model into the Industrial Robotics Simulation Platform.

## 1. Create Description Packages

1. Make a new package for the URDF and SRDF files. Use the naming pattern `<robot_name>_robot_description` and place it under the `src/` directory.
2. Inside the package create `urdf/` and `launch/` folders. Put your robot's URDF (or XACRO) in `urdf/` and provide launch files similar to those in `ur5_robot_description` and `delta_robot_description`.
3. Mesh assets such as `.stl` or `.dae` files should go in a `meshes/` directory inside the description package. Reference them in the URDF using the `package://` URL scheme, for example:
   ```xml
   <mesh filename="package://my_robot_description/meshes/base_link.stl" />
   ```
4. Create a second package named `<robot_name>_moveit_config`. Use the MoveIt Setup Assistant to generate this package. Select your URDF, configure the planning groups, and export the SRDF and configuration files.

## 2. Generate MoveIt Configuration

Run the MoveIt Setup Assistant:
```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```
Load the URDF from your description package and step through the wizard to produce a complete MoveIt package. The generated package should include `launch/` files, `config/kinematics.yaml`, `config/controllers.yaml` and an SRDF.

## 3. Register Launch Files

Both packages require launch files so the robot can be visualized and controlled:
- A display launch file should start `robot_state_publisher`, `joint_state_publisher` (and optionally the GUI) and RViz.
- The MoveIt package must provide a `move_group.launch.py` that loads the URDF and SRDF and launches the `move_group` node.

Use the existing UR5 and delta robot packages as references when creating these launch files.

## 4. YAML Configuration Example

Robots are enabled inside scenario YAML files using the `robots:` section. The following snippet shows typical parameters for a new robot:

```yaml
robots:
  - id: my_robot_arm
    type: my_robot
    position: [0.0, 0.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    parameters:
      speed: 1.0
      acceleration: 0.5
      gripper_type: parallel
```

Place your scenario file in `src/simulation_core/config/` or reference it via the `config_dir` launch argument.
