#!/usr/bin/env python3
"""Launch description bringing up pick and place nodes."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get the package share directory
    fmm_core_pkg_share = get_package_share_directory('fmm_core')
    delta_robot_moveit_config_pkg_share = get_package_share_directory('delta_robot_moveit_config')
    
    # Include the MoveIt2 launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            delta_robot_moveit_config_pkg_share, 
            '/launch/move_group_headless.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Pick and Place node
    pick_and_place_node = Node(
        package='fmm_core',
        executable='pick_and_place_node',
        name='pick_and_place_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': 'delta_robot'},
            {'planning_group': 'delta_arm'},
            {'end_effector_link': 'end_effector'},
            {'base_link': 'base_link'},
            {'detected_objects_topic': '/apm/detection/objects'},
            {'pick_command_topic': '/fmm/pick_command'},
            {'place_command_topic': '/fmm/place_command'},
            {'status_topic': '/fmm/status'},
            {'pre_grasp_distance': 0.1},
            {'post_grasp_distance': 0.1},
            {'planning_time': 5.0},
            {'num_planning_attempts': 10},
            {'max_velocity_scaling_factor': 0.5},
            {'max_acceleration_scaling_factor': 0.5},
            {'workspace_limits': [-0.5, 0.5, -0.5, 0.5, 0.0, 0.5]}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        moveit_launch,
        pick_and_place_node
    ])
