#!/usr/bin/env python3
"""Launch the full simulation system with optional perception features."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    use_realsense = LaunchConfiguration('use_realsense', default='false')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')
    error_simulation_rate = LaunchConfiguration('error_simulation_rate', default='0.0')

    config_dir = os.path.join(
        get_package_share_directory('simulation_core'),
        'config'
    )

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    env_config_node = Node(
        package='simulation_core',
        executable='environment_configurator_node',
        name='environment_configurator',
        parameters=[{
            'config_dir': config_dir,
            'default_scenario': 'default',
            'physics_enabled': True,
            'record_metrics': True,
            'error_simulation_rate': error_simulation_rate,
        }],
        output='screen'
    )

    robot_control_node = Node(
        package='simulation_core',
        executable='robot_control_node',
        name='robot_control',
        output='screen'
    )

    safety_monitor_node = Node(
        package='simulation_core',
        executable='safety_monitor_node',
        name='safety_monitor',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='Enable RealSense camera integration'
        ),
        DeclareLaunchArgument(
            'use_advanced_perception',
            default_value='false',
            description='Enable advanced perception components'
        ),
        DeclareLaunchArgument(
            'error_simulation_rate',
            default_value='0.0',
            description='Probability of injecting a simulation error each cycle'
        ),
        rviz_node,
        env_config_node,
        robot_control_node,
        safety_monitor_node
    ])
