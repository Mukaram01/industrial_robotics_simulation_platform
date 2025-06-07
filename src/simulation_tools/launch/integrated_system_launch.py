#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (LaunchConfiguration, FindExecutable,
                                  PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_realsense = LaunchConfiguration('use_realsense', default='false')
    use_delta_robot = LaunchConfiguration('use_delta_robot', default='true')
    use_ur5_robot = LaunchConfiguration('use_ur5_robot', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    use_moveit = LaunchConfiguration('use_moveit', default='false')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(
        get_package_share_directory('simulation_tools'), 'config'))
    data_dir = LaunchConfiguration('data_dir', default='/tmp/simulation_data')
    allow_unsafe_werkzeug = LaunchConfiguration('allow_unsafe_werkzeug', default='false')
    
    # Create launch configuration arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='Use RealSense camera instead of simulation'),
        DeclareLaunchArgument(
            'use_delta_robot',
            default_value='true',
            description='Use delta robot for sorting'),
        DeclareLaunchArgument(
            'use_ur5_robot',
            default_value='false',
            description='Use UR5 robot for pick and place'),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Launch Gazebo simulation'),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='false',
            description='Launch MoveIt for motion planning'),
        DeclareLaunchArgument(
            'use_advanced_perception',
            default_value='false',
            description='Use advanced perception algorithms'),
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(get_package_share_directory('simulation_tools'), 'config'),
            description='Directory containing configuration files'),
        DeclareLaunchArgument(
            'data_dir',
            default_value='/tmp/simulation_data',
            description='Directory for storing data and exports'),
        DeclareLaunchArgument(
            'allow_unsafe_werkzeug',
            default_value='false',
            description='Allow running the web server using Werkzeug in unsafe mode'),
    ]
    
    # Create data directory during launch execution
    def create_data_dir(context):
        os.makedirs(LaunchConfiguration('data_dir').perform(context), exist_ok=True)
    
    # Define nodes to launch
    nodes = []

    # Include robot descriptions
    delta_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('delta_robot_description'),
                'launch',
                'display_headless.launch.py')),
        condition=IfCondition(use_delta_robot)
    )
    nodes.append(delta_launch)

    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur5_robot_description'),
                'launch',
                'display.launch.py')),
        condition=IfCondition(use_ur5_robot)
    )
    nodes.append(ur5_launch)

    # Optional Gazebo simulation
    simulation_tools_dir = get_package_share_directory('simulation_tools')
    gazebo_launch_file = os.path.join(simulation_tools_dir, 'launch', 'gazebo.launch.py')
    if os.path.exists(gazebo_launch_file):
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file]),
            condition=IfCondition(use_gazebo)
        )
        nodes.append(gazebo_launch)

    # Optional MoveIt integration
    delta_moveit_file = os.path.join(
        get_package_share_directory('delta_robot_moveit_config'),
        'launch', 'move_group_headless.launch.py')
    if os.path.exists(delta_moveit_file):
        delta_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([delta_moveit_file]),
            condition=IfCondition(PythonExpression([use_moveit, " == 'true' and ", use_delta_robot, " == 'true' "]))
        )
        nodes.append(delta_moveit)

    ur5_moveit_file = os.path.join(
        get_package_share_directory('ur5_robot_moveit_config'),
        'launch', 'move_group.launch.py')
    if os.path.exists(ur5_moveit_file):
        ur5_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ur5_moveit_file]),
            condition=IfCondition(PythonExpression([use_moveit, " == 'true' and ", use_ur5_robot, " == 'true' "]))
        )
        nodes.append(ur5_moveit)
    
    # Camera node (either RealSense or simulation)
    if use_realsense == 'true':
        # Launch RealSense camera node
        nodes.append(
            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense2_camera',
                parameters=[{
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'color_width': 640,
                    'color_height': 480,
                    'depth_width': 640,
                    'depth_height': 480,
                    'depth_fps': 30,
                    'color_fps': 30
                }],
                output='screen'
            )
        )
    else:
        # Launch camera simulator node
        nodes.append(
            Node(
                package='simulation_tools',
                executable='camera_simulator_node.py',
                name='camera_simulator',
                parameters=[{
                    'simulation_mode': 'synthetic',
                    'frame_rate': 30.0,
                    'resolution_width': 640,
                    'resolution_height': 480,
                    'camera_name': 'camera',
                    'object_count': 5,
                    'object_types': ['red_cube', 'green_cylinder', 'blue_sphere'],
                    'background_type': 'conveyor_belt',
                    'noise_level': 0.02,
                    'simulate_lighting': True,
                    'simulate_occlusion': False
                }],
                output='screen'
            )
        )
    
    # Environment configurator node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='environment_configurator_node.py',
            name='environment_configurator',
            parameters=[{
                'config_dir': config_dir,
                'default_scenario': 'default',
                'physics_enabled': True,
                'record_metrics': True,
                'error_simulation_rate': 0.0
            }],
            output='screen'
        )
    )
    
    # Web interface node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='web_interface_node.py',
            name='web_interface',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'config_dir': config_dir,
                'data_dir': data_dir,
                'allow_unsafe_werkzeug': allow_unsafe_werkzeug,
            }],
            output='screen'
        )
    )
    
    # Visualization server node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='visualization_server_node.py',
            name='visualization_server',
            parameters=[{
                'data_dir': data_dir,
                'export_enabled': True,
                'export_interval': 60.0,
                'visualization_rate': 10.0
            }],
            output='screen'
        )
    )
    
    # Industrial protocol bridge node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='industrial_protocol_bridge_node.py',
            name='industrial_protocol_bridge',
            parameters=[{
                'config_dir': config_dir,
                'opcua_enabled': True,
                'opcua_endpoint': 'opc.tcp://0.0.0.0:4840/freeopcua/server/',
                'mqtt_enabled': True,
                'mqtt_broker': 'localhost',
                'mqtt_port': 1883,
                'hybrid_mode': use_realsense
            }],
            output='screen'
        )
    )
    
    # Safety monitor node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='safety_monitor_node.py',
            name='safety_monitor',
            parameters=[{
                'config_dir': config_dir,
                'safety_rules_file': 'safety_rules.yaml',
                'emergency_stop_enabled': True,
                'collision_detection_enabled': True,
                'safety_zone_monitoring_enabled': True,
                'check_interval': 0.1
            }],
            output='screen'
        )
    )
    
    # Create launch description with directory initialization
    ld = LaunchDescription(launch_args + [OpaqueFunction(function=create_data_dir)] + nodes)
    
    return ld
