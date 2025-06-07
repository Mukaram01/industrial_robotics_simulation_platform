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
    robot_name = LaunchConfiguration('robot_name', default='delta_robot')
    
    # Get the package share directory
    pkg_share = get_package_share_directory('delta_robot_moveit_config')
    robot_description_pkg_share = get_package_share_directory('delta_robot_description')
    
    # Get the URDF file
    urdf_file = os.path.join(robot_description_pkg_share, 'urdf', 'delta_robot.urdf.xacro')
    
    # Get the SRDF file
    srdf_file = os.path.join(pkg_share, 'config', 'delta_robot.srdf')
    
    # Parse the URDF
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Load kinematics.yaml
    kinematics_yaml = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    
    # Load controllers.yaml
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Robot description parameters
    robot_description = {'robot_description': robot_description_content}
    
    # Robot description semantic parameters
    robot_description_semantic = {
        'robot_description_semantic': Command(['cat ', srdf_file])
    }
    
    # Kinematics parameters
    kinematics_params = {
        'robot_description_kinematics': Command(['cat ', kinematics_yaml])
    }
    
    # Planning parameters
    planning_params = {
        'use_sim_time': use_sim_time,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }
    
    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            planning_params,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz node
    rviz_config = os.path.join(pkg_share, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            planning_params,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
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
            'robot_name',
            default_value='delta_robot',
            description='Robot name'
        ),
        move_group_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
