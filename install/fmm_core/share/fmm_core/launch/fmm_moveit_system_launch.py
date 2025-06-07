#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = LaunchConfiguration('robot_name', default='delta_robot')
    
    # Get the path to the MoveIt2 configuration
    moveit_config_pkg = LaunchConfiguration('moveit_config_pkg', default='delta_robot_moveit_config')
    moveit_config_path = os.path.join(
        get_package_share_directory(moveit_config_pkg),
        'launch'
    )
    
    # MoveIt2 move_group launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_config_path, '/move_group.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name
        }.items()
    )
    
    # Planning scene updater node
    planning_scene_updater_node = Node(
        package='fmm_core',
        executable='planning_scene_updater_node',
        name='planning_scene_updater_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'base_link'},
            {'world_frame': 'world'},
            {'detected_objects_topic': '/apm/detection/objects'},
            {'planning_scene_topic': '/planning_scene'},
            {'update_frequency': 10.0},
            {'object_padding': 0.02}
        ],
        output='screen'
    )
    
    # MoveIt2 interface node
    moveit_interface_node = Node(
        package='fmm_core',
        executable='fmm_moveit_interface_node',
        name='fmm_moveit_interface_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name},
            {'planning_group': 'delta_arm'},
            {'end_effector_link': 'end_effector'},
            {'base_link': 'base_link'},
            {'detected_objects_topic': '/apm/detection/objects'},
            {'pick_command_topic': '/fmm/pick_command'},
            {'place_command_topic': '/fmm/place_command'},
            {'status_topic': '/fmm/status'}
        ],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='delta_robot',
            description='Robot name'
        ),
        DeclareLaunchArgument(
            'moveit_config_pkg',
            default_value='delta_robot_moveit_config',
            description='MoveIt2 configuration package'
        ),
        
        # Nodes and includes
        moveit_launch,
        planning_scene_updater_node,
        moveit_interface_node
    ])
