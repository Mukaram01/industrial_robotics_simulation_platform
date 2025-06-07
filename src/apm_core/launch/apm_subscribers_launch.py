#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/color/image_raw',
        description='Topic for RGB camera images'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_rect_raw',
        description='Topic for depth images'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/camera/depth/color/points',
        description='Topic for point cloud data'
    )
    
    # Image subscriber node
    image_subscriber_node = Node(
        package='apm_core',
        executable='image_subscriber_node',
        name='image_subscriber_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': LaunchConfiguration('camera_topic')
        }],
        output='screen'
    )
    
    # Point cloud subscriber node
    pointcloud_subscriber_node = Node(
        package='apm_core',
        executable='point_cloud_subscriber_node',
        name='point_cloud_subscriber_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic')
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        camera_topic_arg,
        depth_topic_arg,
        pointcloud_topic_arg,
        image_subscriber_node,
        pointcloud_subscriber_node
    ])
