#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_realsense = LaunchConfiguration('use_realsense')
    use_delta_robot = LaunchConfiguration('use_delta_robot')
    use_ur5_robot = LaunchConfiguration('use_ur5_robot')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    
    # Get package directories
    simulation_tools_dir = get_package_share_directory('simulation_tools')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_realsense',
        default_value='true',
        description='Use RealSense camera instead of simulation'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_delta_robot',
        default_value='true',
        description='Use delta robot for sorting'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_ur5_robot',
        default_value='false',
        description='Use UR5 robot for pick and place'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',  # Changed to false since Gazebo files don't exist
        description='Launch Gazebo simulation'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_moveit',
        default_value='false',  # Changed to false since MoveIt files don't exist
        description='Launch MoveIt for motion planning'
    ))
    
    # Launch RViz conditionally using IfCondition
    rviz_config_file = os.path.join(
        simulation_tools_dir, 'config', 'visualization.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    ld.add_action(rviz_node)
    
    # Joint state publisher (always needed for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    ld.add_action(joint_state_publisher)
    
    # Include realsense launch file conditionally
    realsense_launch_file = os.path.join(
        simulation_tools_dir, 'launch', 'realsense_hybrid_launch.py'
    )
    
    if os.path.exists(realsense_launch_file):
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_launch_file]),
            launch_arguments={
                'use_realsense': use_realsense
            }.items(),
            condition=IfCondition(use_realsense)
        )
        ld.add_action(realsense_launch)
    
    # Note: Gazebo and MoveIt launches would go here if the packages existed
    # They would use IfCondition(use_gazebo) and IfCondition(use_moveit)
    
    return ld
