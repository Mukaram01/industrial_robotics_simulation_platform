#!/usr/bin/env python3
"""Launch file combining RealSense data with simulated components."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    # Launch parameters
    config_dir = LaunchConfiguration(
        'config_dir',
        default=os.path.join(
            get_package_share_directory('simulation_core'), 'config')
    )
    camera_config = LaunchConfiguration(
        'camera_config',
        default=os.path.join(
            get_package_share_directory('simulation_core'),
            'config',
            'realsense_config.yaml',
        ),
    )
    data_dir = LaunchConfiguration('data_dir', default='/tmp/simulation_data')
    save_images = LaunchConfiguration('save_images', default='false')
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
            description='Use RealSense camera instead of simulation',
        ),
        DeclareLaunchArgument(
            'use_delta_robot',
            default_value='true',
            description='Use delta robot for sorting',
        ),
        DeclareLaunchArgument(
            'use_advanced_perception',
            default_value='false',
            description='Use advanced perception algorithms',
        ),
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(get_package_share_directory('simulation_core'), 'config'),
            description='Directory containing configuration files'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(
                get_package_share_directory('simulation_core'),
                'config',
                'realsense_config.yaml'),
            description='YAML file with parameters for the RealSense camera'),
        DeclareLaunchArgument(
            'data_dir',
            default_value='/tmp/simulation_data',
            description='Directory for storing data and exports'),
        DeclareLaunchArgument(
            'save_images',
            default_value='false',
            description='Save latest images to disk'),
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
            parameters=[camera_config],
            output='screen'
        )
    )

    # Environment configurator node
    nodes.append(
        Node(
            package='simulation_core',
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
            package='web_interface_backend',
            executable='web_interface_node',
            name='web_interface',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'config_dir': config_dir,
                'data_dir': data_dir,
                'save_images': save_images,
                'allow_unsafe_werkzeug': allow_unsafe_werkzeug,
            }],
            output='screen'
        )
    )

    # Visualization server node
    nodes.append(
        Node(
            package='web_interface_backend',
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
            package='industrial_protocols',
            executable='industrial_protocol_bridge_node',
            name='industrial_protocol_bridge',
            parameters=[{
                'config_dir': config_dir,
                'opcua_enabled': True,
                'opcua_endpoint': ['opc.tcp://127.0.0.1:', opcua_port, '/freeopcua/server/'],
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
            package='simulation_core',
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
