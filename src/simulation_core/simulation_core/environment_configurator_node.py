#!/usr/bin/env python3
"""ROS2 node loading environment configs and spawning scenarios."""

import os
import re
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml  # type: ignore
from typing import Any
import json
import threading
import time
import random

SCENARIO_ID_PATTERN = re.compile(r'^[A-Za-z0-9_]+$')

class EnvironmentConfiguratorNode(Node):
    def __init__(self) -> None:
        super().__init__('environment_configurator_node')
        
        # Declare parameters using a single dictionary
        param_defaults = {
            'config_dir': '',
            'default_scenario': 'default',
            # Optional scenario to load at startup. If empty, default_scenario is used.
            'scenario': '',
            'physics_enabled': True,
            'record_metrics': True,
            # Probability (0-1) of triggering a random error during metrics update
            'error_simulation_rate': 0.0,
        }
        self.declare_parameters('', [(k, v) for k, v in param_defaults.items()])

        # Get parameters
        self.config_dir = self.get_parameter('config_dir').value
        self.default_scenario = self.get_parameter('default_scenario').value
        self.scenario = self.get_parameter('scenario').value
        self.physics_enabled = self.get_parameter('physics_enabled').value
        self.record_metrics = self.get_parameter('record_metrics').value
        self.error_simulation_rate = self.get_parameter('error_simulation_rate').value

        # Initialize state
        self.current_scenario = self.scenario if self.scenario else self.default_scenario
        self.running = False
        self.environment_config: dict[str, Any] = {}
        self._threads: list[threading.Thread] = []
        self.load_scenario(self.current_scenario)

        # Create publishers
        self.status_pub = self.create_publisher(
            String, 
            '/simulation/status', 
            10)
        self.metrics_pub = self.create_publisher(
            String, 
            '/simulation/metrics', 
            10)
        
        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            '/simulation/command',
            self.command_callback,
            10)
        self.config_sub = self.create_subscription(
            String,
            '/simulation/config',
            self.config_callback,
            10)
        
        # Start status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start metrics timer if enabled
        if self.record_metrics:
            self.metrics_timer = self.create_timer(0.5, self.publish_metrics)
            self.metrics = {
                'cycle_time': 0.0,
                'throughput': 0.0,
                'accuracy': 100.0,
                'errors': 0,
                'objects_processed': 0
            }
            self.last_metrics_time = time.time()
        
        self.get_logger().info('Environment configurator node initialized')
    
    def command_callback(self, msg: String) -> None:
        command = msg.data
        
        if command == 'start':
            self.running = True
            self.get_logger().info('Starting simulation')
            
            # Reset metrics
            if self.record_metrics:
                self.metrics = {
                    'cycle_time': 0.0,
                    'throughput': 0.0,
                    'accuracy': 100.0,
                    'errors': 0,
                    'objects_processed': 0
                }
                self.last_metrics_time = time.time()
        
        elif command == 'stop':
            self.running = False
            self.get_logger().info('Stopping simulation')
        
        elif command == 'reset':
            self.get_logger().info('Resetting simulation')
            self.load_scenario(self.current_scenario)
            
            # Reset metrics
            if self.record_metrics:
                self.metrics = {
                    'cycle_time': 0.0,
                    'throughput': 0.0,
                    'accuracy': 100.0,
                    'errors': 0,
                    'objects_processed': 0
                }
                self.last_metrics_time = time.time()
        
        elif command == 'emergency_stop':
            self.running = False
            self.get_logger().warning('Emergency stop triggered')
        
        elif command == 'start_recording':
            self.get_logger().info('Starting recording')
            # In a real implementation, this would start recording the simulation
        
        elif command == 'stop_recording':
            self.get_logger().info('Stopping recording')
            # In a real implementation, this would stop recording and save
        
        elif command == 'start_playback':
            self.get_logger().info('Starting playback')
            # In a real implementation, this would start playback
        
        elif command == 'stop_playback':
            self.get_logger().info('Stopping playback')
            # In a real implementation, this would stop playback
    
    def config_callback(self, msg: String) -> None:
        try:
            config_data = json.loads(msg.data)
            
            # Handle scenario change
            if 'scenario' in config_data:
                scenario = config_data['scenario']
                self.get_logger().info(f'Changing scenario to: {scenario}')
                self.load_scenario(scenario)
            
            # Handle settings update
            if 'settings' in config_data:
                settings = config_data['settings']
                self.update_settings(settings)
            
            # Handle save scenario
            if 'save_scenario' in config_data:
                scenario_data = config_data['save_scenario']
                self.save_scenario(scenario_data)

            # Handle update scenario
            if 'update_scenario' in config_data:
                scenario_data = config_data['update_scenario']
                self.current_scenario = scenario_data.get('name', self.current_scenario)
                self.environment_config = scenario_data.get('config', {})
                self.save_scenario({
                    'name': self.current_scenario,
                    'description': scenario_data.get('description', 'Edited scenario'),
                    'config': self.environment_config,
                })
            
            # Handle delete scenario
            if 'delete_scenario' in config_data:
                scenario_id = config_data['delete_scenario']
                self.delete_scenario(scenario_id)
            
            # Handle environment update
            if 'environment' in config_data:
                self.environment_config['environment'] = config_data['environment']
            
            # Handle add object
            if 'add_object' in config_data:
                obj = config_data['add_object']
                if 'objects' not in self.environment_config:
                    self.environment_config['objects'] = []
                self.environment_config['objects'].append(obj)
            
            # Handle update object
            if 'update_object' in config_data:
                obj = config_data['update_object']
                if 'objects' in self.environment_config:
                    for i, existing_obj in enumerate(self.environment_config['objects']):
                        if existing_obj.get('id') == obj.get('id'):
                            self.environment_config['objects'][i].update(obj)
                            break
            
            # Handle remove object
            if 'remove_object' in config_data:
                obj_id = config_data['remove_object']
                if 'objects' in self.environment_config:
                    self.environment_config['objects'] = [
                        obj for obj in self.environment_config['objects'] 
                        if obj.get('id') != obj_id
                    ]
            
            # Handle error simulation
            if config_data.get('action') == 'trigger_error':
                error_type = config_data.get('type', 'generic')
                self.simulate_error(error_type)
        
        except Exception as e:
            self.get_logger().error(f'Error processing config message: {e}')
    
    def load_scenario(self, scenario: str) -> None:
        if not SCENARIO_ID_PATTERN.match(scenario):
            self.get_logger().warning(f'Invalid scenario ID: {scenario}')
            return

        self.current_scenario = scenario
        
        # Try to load scenario from file
        if self.config_dir:
            yaml_path = os.path.join(self.config_dir, f'{scenario}.yaml')
            json_path = os.path.join(self.config_dir, f'{scenario}.json')

            for scenario_path in (yaml_path, json_path):
                if os.path.exists(scenario_path):
                    try:
                        with open(scenario_path, "r") as f:
                            if scenario_path.endswith(".json"):
                                self.environment_config = json.load(f) or {}
                            else:
                                self.environment_config = yaml.safe_load(f) or {}
                        self.get_logger().info(
                            f"Loaded scenario from {scenario_path}"
                        )
                    except Exception as e:
                        self.get_logger().error(
                            f"Error loading scenario file {scenario}: {e}"
                        )
                    else:
                        self._load_robot_models()
                        return
        
        # If file not found or error, use default configuration
        self.environment_config = self.get_default_config()
        self.get_logger().info('Using default configuration')
        self._load_robot_models()
    
    def save_scenario(self, scenario_data: dict) -> None:
        if not self.config_dir:
            self.get_logger().error('Config directory not set, cannot save scenario')
            return

        os.makedirs(self.config_dir, exist_ok=True)

        scenario_id = scenario_data.get('name', 'custom')
        if not SCENARIO_ID_PATTERN.match(scenario_id):
            self.get_logger().warning(f'Invalid scenario ID: {scenario_id}')
            return
        description = scenario_data.get('description', 'Custom scenario')
        config = scenario_data.get('config', {})
        
        # Add description if not present
        if 'description' not in config:
            config['description'] = description
        
        # Save to file
        scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
        
        try:
            with open(scenario_path, 'w') as f:
                yaml.safe_dump(config, f, default_flow_style=False)
            self.get_logger().info(f'Saved scenario to {scenario_path}')
        except Exception as e:
            self.get_logger().error(f'Error saving scenario file {scenario_id}: {e}')
    
    def delete_scenario(self, scenario_id: str) -> None:
        if not self.config_dir:
            self.get_logger().error('Config directory not set, cannot delete scenario')
            return

        if not SCENARIO_ID_PATTERN.match(scenario_id):
            self.get_logger().warning(f'Invalid scenario ID: {scenario_id}')
            return
        
        # Don't delete default scenario
        if scenario_id == 'default':
            self.get_logger().warning('Cannot delete default scenario')
            return
        
        scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
        json_path = os.path.join(self.config_dir, f'{scenario_id}.json')

        for path in (scenario_path, json_path):
            if os.path.exists(path):
                try:
                    os.remove(path)
                    self.get_logger().info(f'Deleted scenario {scenario_id}')
                except Exception as e:
                    self.get_logger().error(f'Error deleting scenario file {scenario_id}: {e}')

    def _launch_process(self, cmd: list[str]) -> None:
        """Launch a ROS2 process if available."""
        if shutil.which(cmd[0]) is None:
            self.get_logger().warning(f'{cmd[0]} not found, skipping process')
            return
        thread = threading.Thread(
            target=subprocess.run,
            args=(cmd,),
            kwargs={'check': False},
        )
        thread.start()
        self._threads.append(thread)

    def _load_robot_models(self) -> None:
        """Parse robot model files and start appropriate nodes."""
        robots = self.environment_config.get('robots', [])
        for robot in robots:
            model_file = robot.get('model_file')
            if not model_file:
                self.get_logger().warning(
                    f"Robot {robot.get('id', '?')} missing model_file"
                )
                continue

            if model_file.endswith(('.urdf', '.urdf.xacro')):
                try:
                    from urdf_parser_py.urdf import URDF
                    URDF.from_xml_file(model_file)
                    self.get_logger().info(
                        f"Launching robot_state_publisher for {model_file}"
                    )
                    self._launch_process(
                        [
                            'ros2',
                            'run',
                            'robot_state_publisher',
                            'robot_state_publisher',
                            model_file,
                        ]
                    )
                except Exception as e:
                    self.get_logger().error(f'Failed to parse URDF {model_file}: {e}')
            elif model_file.endswith('.sdf'):
                try:
                    import sdformat  # noqa: F401
                except Exception:
                    self.get_logger().warning(
                        'sdformat package not available, cannot spawn SDF models'
                    )
                    continue
                self.get_logger().info(
                    f"Spawning Gazebo entity from {model_file}"
                )
                self._launch_process(
                    [
                        'ros2',
                        'run',
                        'gazebo_ros',
                        'spawn_entity.py',
                        '-file',
                        model_file,
                        '-entity',
                        robot.get('id', 'robot'),
                    ]
                )
            else:
                self.get_logger().warning(
                    f"Unsupported model file extension: {model_file}"
                )
    
    def update_settings(self, settings: dict) -> None:
        if 'simulation' in settings:
            sim_settings = settings['simulation']
            self.physics_enabled = sim_settings.get('physics_enabled', self.physics_enabled)
            self.record_metrics = sim_settings.get('record_metrics', self.record_metrics)
            # Value should be between 0 and 1
            rate = sim_settings.get('error_simulation_rate', self.error_simulation_rate)
            self.error_simulation_rate = max(0.0, min(rate, 1.0))
        
        self.get_logger().info('Updated settings')
    
    def simulate_error(self, error_type: str) -> None:
        if not self.running:
            return
        
        self.get_logger().info(f'Simulating error: {error_type}')
        
        # Update metrics
        if self.record_metrics:
            self.metrics['errors'] += 1
            
            # Different error types affect different metrics
            if error_type == 'gripper_failure':
                self.metrics['accuracy'] = max(0.0, self.metrics['accuracy'] - 5.0)
            elif error_type == 'object_slip':
                self.metrics['accuracy'] = max(0.0, self.metrics['accuracy'] - 3.0)
            elif error_type == 'sensor_noise':
                self.metrics['accuracy'] = max(0.0, self.metrics['accuracy'] - 2.0)
    
    def publish_status(self) -> None:
        status_msg = String()
        status_data = {
            'status': 'running' if self.running else 'idle',
            'current_scenario': self.current_scenario,
            'physics_enabled': self.physics_enabled,
            'error_simulation_rate': self.error_simulation_rate
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def publish_metrics(self) -> None:
        if not self.running or not self.record_metrics:
            return
        
        # Update metrics based on simulation time
        current_time = time.time()
        elapsed = current_time - self.last_metrics_time
        self.last_metrics_time = current_time
        
        # Simulate some activity
        if self.running:
            # Update cycle time (simulated)
            self.metrics['cycle_time'] = 2.0 + 0.5 * (time.time() % 2)
            
            # Update throughput (simulated)
            new_objects = int(elapsed * 5)  # 5 objects per second
            self.metrics['objects_processed'] += new_objects
            self.metrics['throughput'] = 60.0 * new_objects / elapsed if elapsed > 0 else 0
            
            # Randomly trigger an error based on the configured rate
            if self.error_simulation_rate > 0 and random.random() < self.error_simulation_rate:
                self.simulate_error('random')
        
        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(self.metrics)
        self.metrics_pub.publish(metrics_msg)

    def shutdown(self) -> None:
        """Cancel timers and join spawned threads."""
        try:
            self.status_timer.cancel()
        except Exception:  # pragma: no cover - best effort
            pass
        if self.record_metrics:
            try:
                self.metrics_timer.cancel()
            except Exception:  # pragma: no cover - best effort
                pass
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=0.1)
    
    def get_default_config(self) -> dict:
        return {
            'description': 'Default simulation scenario',
            'environment': {
                'type': 'factory',
                'size': [10.0, 10.0, 3.0],
                'gravity': [0.0, 0.0, -9.81],
                'ambient_light': 0.7
            },
            'robots': [
                {
                    'id': 'delta_robot_1',
                    'type': 'delta_robot',
                    'model_file': 'src/delta_robot_description/urdf/delta_robot.urdf.xacro',
                    'position': [0.0, 0.0, 1.5],
                    'orientation': [0.0, 0.0, 0.0, 1.0],
                    'parameters': {
                        'speed': 1.0,
                        'acceleration': 1.0,
                        'workspace_radius': 0.5,
                        'gripper_type': 'vacuum'
                    }
                }
            ],
            'conveyors': [
                {
                    'id': 'conveyor_1',
                    'type': 'belt',
                    'position': [0.0, 0.0, 0.5],
                    'orientation': [0.0, 0.0, 0.0, 1.0],
                    'dimensions': [2.0, 0.5, 0.1],
                    'speed': 0.2,
                    'direction': [0.0, 1.0, 0.0]
                }
            ],
            'containers': [
                {
                    'id': 'bin_red',
                    'type': 'bin',
                    'position': [0.5, 0.5, 0.0],
                    'dimensions': [0.3, 0.3, 0.2],
                    'color': [1.0, 0.0, 0.0]
                },
                {
                    'id': 'bin_green',
                    'type': 'bin',
                    'position': [0.5, -0.5, 0.0],
                    'dimensions': [0.3, 0.3, 0.2],
                    'color': [0.0, 1.0, 0.0]
                },
                {
                    'id': 'bin_blue',
                    'type': 'bin',
                    'position': [-0.5, 0.5, 0.0],
                    'dimensions': [0.3, 0.3, 0.2],
                    'color': [0.0, 0.0, 1.0]
                }
            ]
        }

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EnvironmentConfiguratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
