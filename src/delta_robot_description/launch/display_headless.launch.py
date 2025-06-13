#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get the package share directory
    pkg_share = get_package_share_directory('delta_robot_description')
    
    # Get the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'delta_robot.urdf.xacro')
    
    # Parse the URDF once so other nodes (for example RViz if launched
    # separately) can receive the same robot description when a namespace is
    # provided.
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # Joint state publisher node (headless version)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        robot_state_publisher,
        joint_state_publisher
    ])
