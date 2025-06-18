#!/usr/bin/env python3
"""Script that creates a default simulation configuration YAML file."""

import os
import yaml

# Default configuration for delta robot sorting scenario
default_config = {
    "description": "Delta robot sorting objects from conveyor belt",
    "environment": {
        "type": "factory",
        "size": [10.0, 10.0, 3.0],
        "gravity": [0.0, 0.0, -9.81],
        "ambient_light": 0.7
    },
    "robots": [
        {
            "id": "delta_robot_1",
            "type": "delta_robot",
            "model_file": "../delta_robot_description/urdf/delta_robot.urdf.xacro",
            "position": [0.0, 0.0, 1.5],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "parameters": {
                "speed": 1.0,
                "acceleration": 1.0,
                "workspace_radius": 0.5,
                "gripper_type": "vacuum"
            }
        }
    ],
    "conveyors": [
        {
            "id": "conveyor_1",
            "type": "belt",
            "position": [0.0, 0.0, 0.5],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "dimensions": [2.0, 0.5, 0.1],
            "speed": 0.2,
            "direction": [0.0, 1.0, 0.0]
        }
    ],
    "containers": [
        {
            "id": "bin_red",
            "type": "bin",
            "position": [0.5, 0.5, 0.0],
            "dimensions": [0.3, 0.3, 0.2],
            "color": [1.0, 0.0, 0.0]
        },
        {
            "id": "bin_green",
            "type": "bin",
            "position": [0.5, -0.5, 0.0],
            "dimensions": [0.3, 0.3, 0.2],
            "color": [0.0, 1.0, 0.0]
        },
        {
            "id": "bin_blue",
            "type": "bin",
            "position": [-0.5, 0.5, 0.0],
            "dimensions": [0.3, 0.3, 0.2],
            "color": [0.0, 0.0, 1.0]
        }
    ],
    "sorting_rules": [
        {
            "object_type": "red_cube",
            "destination": "bin_red"
        },
        {
            "object_type": "green_cylinder",
            "destination": "bin_green"
        },
        {
            "object_type": "blue_sphere",
            "destination": "bin_blue"
        }
    ]
}

# Create config directory if it doesn't exist
os.makedirs(os.path.dirname(os.path.abspath(__file__)), exist_ok=True)

# Write default configuration to file
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'default.yaml'), 'w') as f:
    yaml.safe_dump(default_config, f, default_flow_style=False)

print("Default configuration created successfully.")
