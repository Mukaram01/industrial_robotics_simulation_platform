#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, OpaqueFunction,
                            IncludeLaunchDescription)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_realsense = LaunchConfiguration('use_realsense', default='false')
    use_delta_robot = LaunchConfiguration('use_delta_robot', default='true')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')
    scenario = LaunchConfiguration('scenario', default='default')
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(
        get_package_share_directory('simulation_tools'), 'config'))
    data_dir = LaunchConfiguration('data_dir', default='/tmp/simulation_data')
    allow_unsafe_werkzeug = LaunchConfiguration('allow_unsafe_werkzeug', default='false')
    opcua_port = LaunchConfiguration('opcua_port', default='4840')
    
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
            'use_advanced_perception',
            default_value='false',
            description='Use advanced perception algorithms'),
        DeclareLaunchArgument(
            'scenario',
            default_value='default',
            description='Scenario configuration to load'),
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
        DeclareLaunchArgument(
            'opcua_port',
            default_value='4840',
            description='Port for the OPC UA server'),
    ]
    
    # Create data directory during launch execution
    def create_data_dir(context):
        os.makedirs(LaunchConfiguration('data_dir').perform(context), exist_ok=True)
    
    # Define nodes to launch
    nodes = []
    
    # Camera node (either RealSense or simulation)
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
            output='screen',
            condition=IfCondition(use_realsense)
        )
    )
    nodes.append(
        Node(
            package='simulation_tools',
            executable='camera_simulator_node',
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
            output='screen',
            condition=UnlessCondition(use_realsense)
        )
    )
    
    # Environment configurator node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='environment_configurator_node',
            name='environment_configurator',
            parameters=[{
                'config_dir': config_dir,
                'default_scenario': scenario,
                'physics_enabled': True,
                'record_metrics': True,
                'error_simulation_rate': 0.0
            }],
            output='screen'
        )
    )

    # Advanced perception stack
    advanced_perception_pkg_share = get_package_share_directory('advanced_perception')
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    advanced_perception_pkg_share,
                    'launch',
                    'advanced_perception_launch.py'
                )
            ),
            condition=IfCondition(use_advanced_perception)
        )
    )
    
    # Web interface node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='web_interface_node',
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
            executable='visualization_server_node',
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
            executable='industrial_protocol_bridge_node',
            name='industrial_protocol_bridge',
            parameters=[{
                'config_dir': config_dir,
                'opcua_enabled': True,
                'opcua_endpoint': ['opc.tcp://0.0.0.0:', opcua_port, '/freeopcua/server/'],
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
            executable='safety_monitor_node',
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
