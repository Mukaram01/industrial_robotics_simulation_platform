#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

# Nodes launched here use MultiThreadedExecutor internally
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_realsense = LaunchConfiguration('use_realsense', default='true')
    use_delta_robot = LaunchConfiguration('use_delta_robot', default='true')
    use_advanced_perception = LaunchConfiguration('use_advanced_perception', default='false')
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(
        get_package_share_directory('simulation_tools'), 'config'))
    data_dir = LaunchConfiguration('data_dir', default='/tmp/simulation_data')
    allow_unsafe_werkzeug = LaunchConfiguration('allow_unsafe_werkzeug', default='true')
    opcua_port = LaunchConfiguration('opcua_port', default='4840')
    
    # Create default data directory (using the default value, not the LaunchConfiguration)
    default_data_dir = '/tmp/simulation_data'
    os.makedirs(default_data_dir, exist_ok=True)
    
    # Create launch configuration arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
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
            'config_dir',
            default_value=os.path.join(get_package_share_directory('simulation_tools'), 'config'),
            description='Directory containing configuration files'),
        DeclareLaunchArgument(
            'data_dir',
            default_value='/tmp/simulation_data',
            description='Directory for storing data and exports'),
        DeclareLaunchArgument(
            'allow_unsafe_werkzeug',
            default_value='true',
            description='Allow running the web server using Werkzeug in unsafe mode'),
        DeclareLaunchArgument(
            'opcua_port',
            default_value='4840',
            description='Port for the OPC UA server'),
    ]
    
    # Define nodes to launch
    nodes = []
    
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
    
    # Environment configurator node
    nodes.append(
        Node(
            package='simulation_tools',
            executable='environment_configurator_node',
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
                'hybrid_mode': True
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
    
    # Create launch description
    ld = LaunchDescription(launch_args + nodes)
    
    return ld
