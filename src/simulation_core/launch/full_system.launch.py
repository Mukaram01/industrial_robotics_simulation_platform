#!/usr/bin/env python3
"""Launch the full simulation system with optional perception features."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_realsense = LaunchConfiguration('use_realsense', default='false')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    node1 = Node(
        package='package1',
        executable='node1',
        name='node1',
        parameters=[{'use_realsense': use_realsense}]
    )

    node2 = Node(
        package='package2',
        executable='node2',
        name='node2',
        parameters=[{'use_advanced_perception': use_advanced_perception}]
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
        rviz_node,
        node1,
        node2
    ])
