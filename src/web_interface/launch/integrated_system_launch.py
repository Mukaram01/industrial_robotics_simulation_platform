#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_delta_robot = LaunchConfiguration('use_delta_robot', default='true')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')
    
    # Get the package share directories
    web_interface_pkg_share = get_package_share_directory('web_interface')
    fmm_core_pkg_share = get_package_share_directory('fmm_core')
    apm_core_pkg_share = get_package_share_directory('apm_core')
    advanced_perception_pkg_share = get_package_share_directory('advanced_perception')
    
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
    
    # Include the sorting demo launch file from fmm_core
    sorting_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            fmm_core_pkg_share, 
            '/launch/sorting_demo_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Conditionally include advanced perception launch
    advanced_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            advanced_perception_pkg_share, 
            '/launch/advanced_perception_launch.py'
        ]),
        condition=launch.conditions.IfCondition(use_advanced_perception),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_delta_robot',
            default_value='true',
            description='Use delta robot instead of UR5'
        ),
        DeclareLaunchArgument(
            'use_advanced_perception',
            default_value='false',
            description='Use advanced perception module'
        ),
        web_interface_node,
        sorting_demo_launch,
        advanced_perception_launch
    ])
