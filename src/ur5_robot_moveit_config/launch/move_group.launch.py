#!/usr/bin/env python3
"""Launch MoveIt2 for the UR5 robot."""

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ur5_robot_moveit_config')
    ur5_description_share = get_package_share_directory('ur5_robot_description')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load URDF and SRDF files
    urdf_file = os.path.join(ur5_description_share, 'urdf', 'ur5_robot.urdf.xacro')
    srdf_file = os.path.join(pkg_share, 'config', 'ur5_robot.srdf')
    
    # Load configuration files
    kinematics_yaml = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': launch.substitutions.Command(['xacro ', urdf_file])}
        ]
    )
    
    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # MoveIt2 move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': launch.substitutions.Command(['xacro ', urdf_file])},
            {'robot_description_semantic': open(srdf_file).read()},
            {'kinematics_yaml': kinematics_yaml},
            {'controllers_yaml': controllers_yaml},
            {'allow_trajectory_execution': True},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
            {'monitor_dynamics': False}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        robot_state_publisher,
        joint_state_publisher,
        move_group_node
    ])
