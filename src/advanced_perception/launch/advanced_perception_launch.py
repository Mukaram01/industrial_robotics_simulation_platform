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
    pkg_share = get_package_share_directory('advanced_perception')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load configuration files
    segmentation_config = os.path.join(pkg_share, 'config', 'segmentation_config.yaml')
    pose_estimation_config = os.path.join(pkg_share, 'config', 'pose_estimation_config.yaml')
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Nodes
        Node(
            package='advanced_perception',
            executable='segmentation_node',
            name='segmentation_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'segmentation_config': segmentation_config}
            ]
        ),
        
        Node(
            package='advanced_perception',
            executable='pose_estimation_node',
            name='pose_estimation_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'pose_estimation_config': pose_estimation_config}
            ]
        )
    ])
