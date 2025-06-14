#!/usr/bin/env python3
"""Launch file that loads a YAML sorting configuration."""
import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file')
    
    # Get the package share directory
    fmm_core_pkg_share = get_package_share_directory('fmm_core')
    
    # Default config file path
    default_config_path = os.path.join(fmm_core_pkg_share, 'config', 'default_sorting_config.yaml')
    
    # Sorting Configuration Loader node
    config_loader_node = Node(
        package='fmm_core',
        executable='config_loader_node',
        name='sorting_config_loader',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': config_file}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Path to the sorting configuration YAML file'
        ),
        config_loader_node
    ])
