# Robot Integration Guide

This guide outlines the basic steps for adding a new robot to the simulation platform.

## 1. Create a Description Package

1. Start by creating a new package for your robot's URDF/Xacro files.
2. Copy the templates located in `templates/robot_description` to your package:

```bash
cp templates/robot_description/template_robot.urdf.xacro <your_pkg>/urdf/
cp templates/robot_description/template_robot.srdf <your_pkg>/config/
```

3. Edit the copied files and replace the placeholder comments with your robot's link and joint definitions.

## 2. Update Launch and Configuration Files

After defining the robot, update your launch files and MoveIt configuration to reference the new description files.

Refer to the existing robot packages in `src/` for examples of how everything fits together.
