#!/usr/bin/env python3
"""Launch file to run the ONNX-based object detection pipeline."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default_object_detection_config.yaml',
        description='Configuration file for object detection'
    )
    
    # Get the path to the config directory
    config_dir = os.path.join(
        get_package_share_directory('apm_core'),
        'config'
    )
    
    # Update the default config to use our newly converted model
    object_detection_config = {
        'model': {
            'path': os.path.join(get_package_share_directory('apm_core'), 'models', 'ssd_mobilenet_v2.onnx'),
            'input_name': 'input_tensor',
            'output_names': ['detection_boxes', 'detection_scores', 'detection_classes'],
            'input_size': [320, 320],
            'confidence_threshold': 0.5
        },
        'classes': {
            'path': os.path.join(get_package_share_directory('apm_core'), 'config', 'coco_classes.txt')
        },
        'processing': {
            'enable_gpu': False,
            'execution_provider': 'CPUExecutionProvider',
            'max_detections': 10
        },
        'camera': {
            'topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/color/camera_info'
        },
        'output': {
            'publish_visualization': True,
            'visualization_topic': '/apm/detection/visualization',
            'detections_topic': '/apm/detection/detections',
            'detected_objects_topic': '/apm/detection/objects'
        }
    }
    
    # ONNX inference node
    onnx_inference_node = Node(
        package='apm_core',
        executable='onnx_inference_node',
        name='onnx_inference_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': LaunchConfiguration('config_file')},
            object_detection_config
        ],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        config_file_arg,
        onnx_inference_node
    ])
