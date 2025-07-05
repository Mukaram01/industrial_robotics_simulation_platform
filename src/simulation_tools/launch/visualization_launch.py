#!/usr/bin/env python3
"""Launch description for visualization tools such as RViz."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_realsense = LaunchConfiguration("use_realsense")
    use_rviz = LaunchConfiguration("use_rviz")
    use_delta_robot = LaunchConfiguration("use_delta_robot")
    use_ur5_robot = LaunchConfiguration("use_ur5_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Get package directories
    simulation_tools_dir = get_package_share_directory('simulation_tools')
    delta_desc_dir = get_package_share_directory('delta_robot_description')
    ur5_desc_dir = get_package_share_directory('ur5_robot_description')

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

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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

    # Robot descriptions
    delta_urdf = os.path.join(delta_desc_dir, 'urdf', 'delta_robot.urdf.xacro')
    ur5_urdf = os.path.join(ur5_desc_dir, 'urdf', 'ur5_robot.urdf.xacro')

    delta_description = Command(['xacro', delta_urdf])
    ur5_description = Command(['xacro', ur5_urdf])

    # Robot state publishers
    delta_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='delta_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': delta_description,
        }],
        condition=IfCondition(use_delta_robot),
    )
    ld.add_action(delta_state_publisher)

    ur5_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ur5_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ur5_description,
        }],
        condition=IfCondition(use_ur5_robot),
    )
    ld.add_action(ur5_state_publisher)

    # Joint state publishers
    delta_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='delta_joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_delta_robot),
    )
    ld.add_action(delta_joint_state_publisher)

    ur5_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='ur5_joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ur5_robot),
    )
    ld.add_action(ur5_joint_state_publisher)

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
