#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get the package share directory
    fmm_core_pkg_share = get_package_share_directory('fmm_core')
    apm_core_pkg_share = get_package_share_directory('apm_core')
    
    # Include the object detection launch file
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            apm_core_pkg_share, 
            '/launch/object_detection_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Include the pick and place launch file
    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            fmm_core_pkg_share, 
            '/launch/pick_and_place_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Sorting Demo node
    sorting_demo_node = Node(
        package='fmm_core',
        executable='sorting_demo_node',
        name='sorting_demo_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'pick_command_topic': '/fmm/pick_command'},
            {'place_command_topic': '/fmm/place_command'},
            {'status_topic': '/fmm/status'},
            {'sorting_locations': {
                'red': [0.3, 0.3, 0.1],
                'green': [0.3, -0.3, 0.1],
                'blue': [-0.3, 0.3, 0.1],
                'unknown': [-0.3, -0.3, 0.1]
            }}
        ]
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        object_detection_launch,
        pick_and_place_launch,
        sorting_demo_node
    ])
