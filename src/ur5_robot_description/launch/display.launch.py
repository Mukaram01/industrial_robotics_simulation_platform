#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ur5_robot_description')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5_robot.urdf.xacro')
    
    # Generate robot description once so both RViz and the state publisher
    # receive the same parameter even when a namespace is used.
    robot_description_content = launch.substitutions.Command(['xacro ', urdf_file])

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content}
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

    # RViz2 node
    rviz_config = os.path.join(pkg_share, 'config', 'ur5.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content}
        ],
        arguments=['-d', rviz_config]
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
        rviz2
    ])
