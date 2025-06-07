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
    pkg_share = get_package_share_directory('web_interface')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Web interface node
    web_interface_node = Node(
        package='web_interface',
        executable='web_interface_node',
        name='web_interface_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'port': 5000},
            {'host': '0.0.0.0'}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        web_interface_node
    ])
