#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
import yaml
import json
import threading
import time

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Declare parameters
        self.declare_parameter('config_dir', '')
        self.declare_parameter('safety_rules_file', 'safety_rules.yaml')
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('collision_detection_enabled', True)
        self.declare_parameter('safety_zone_monitoring_enabled', True)
        self.declare_parameter('check_interval', 0.1)
        
        # Get parameters
        self.config_dir = self.get_parameter('config_dir').value
        self.safety_rules_file = self.get_parameter('safety_rules_file').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        self.collision_detection_enabled = self.get_parameter('collision_detection_enabled').value
        self.safety_zone_monitoring_enabled = self.get_parameter('safety_zone_monitoring_enabled').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # Initialize state
        self.environment_config = {}
        self.safety_rules = self.load_safety_rules()
        self.emergency_stop_active = False
        self.safety_violations = []
        
        # Create publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool, 
            '/safety/emergency_stop', 
            10)
        self.status_pub = self.create_publisher(
            String, 
            '/safety/status', 
            10)
        self.command_pub = self.create_publisher(
            String, 
            '/simulation/command', 
            10)
        
        # Create subscribers
        self.config_sub = self.create_subscription(
            String,
            '/simulation/config',
            self.config_callback,
            10)
        self.command_sub = self.create_subscription(
            String,
            '/simulation/command',
            self.command_callback,
            10)
        
        # Start safety check timer
        self.safety_timer = self.create_timer(self.check_interval, self.safety_check_callback)
        
        self.get_logger().info('Safety monitor node initialized')
    
    def load_safety_rules(self):
        # Default safety rules
        default_rules = {
            'collision_detection': {
                'enabled': True,
                'min_distance': 0.1,
                'objects': ['robot', 'human', 'obstacle']
            },
            'safety_zones': [
                {
                    'name': 'robot_workspace',
                    'type': 'cylinder',
                    'center': [0.0, 0.0, 0.0],
                    'radius': 1.0,
                    'height': 2.0,
                    'restricted_objects': ['human']
                }
            ],
            'emergency_stop': {
                'auto_triggers': ['collision', 'zone_violation', 'speed_violation'],
                'reset_requires_confirmation': True
            }
        }
        
        # Try to load rules from file
        if self.config_dir and self.safety_rules_file:
            rules_path = os.path.join(self.config_dir, self.safety_rules_file)
            
            if os.path.exists(rules_path):
                try:
                    with open(rules_path, 'r') as f:
                        rules = yaml.safe_load(f)
                        self.get_logger().info(f'Loaded safety rules from {rules_path}')
                        return rules
                except Exception as e:
                    self.get_logger().error(f'Error loading safety rules file: {e}')
        
        self.get_logger().info('Using default safety rules')
        return default_rules
    
    def config_callback(self, msg):
        try:
            config_data = json.loads(msg.data)
            if 'environment' in config_data:
                self.environment_config = config_data['environment']
            
            # Check for safety settings
            if 'settings' in config_data and 'safety' in config_data['settings']:
                safety_settings = config_data['settings']['safety']
                self.emergency_stop_enabled = safety_settings.get('emergency_stop_enabled', self.emergency_stop_enabled)
                self.collision_detection_enabled = safety_settings.get('collision_detection_enabled', self.collision_detection_enabled)
                self.safety_zone_monitoring_enabled = safety_settings.get('safety_zone_monitoring_enabled', self.safety_zone_monitoring_enabled)
                
                # Update min_distance if provided
                if 'min_distance' in safety_settings and 'collision_detection' in self.safety_rules:
                    self.safety_rules['collision_detection']['min_distance'] = safety_settings['min_distance']
        except Exception as e:
            self.get_logger().error(f'Error processing config message: {e}')
    
    def command_callback(self, msg):
        command = msg.data
        
        if command == 'emergency_stop':
            self.trigger_emergency_stop('manual')
        elif command == 'reset_emergency_stop':
            self.reset_emergency_stop()
    
    def safety_check_callback(self):
        if not self.emergency_stop_enabled:
            return
        
        violations = []
        
        # Check for collisions
        if self.collision_detection_enabled and 'collision_detection' in self.safety_rules:
            collision_violations = self.check_collisions()
            violations.extend(collision_violations)
        
        # Check safety zones
        if self.safety_zone_monitoring_enabled and 'safety_zones' in self.safety_rules:
            zone_violations = self.check_safety_zones()
            violations.extend(zone_violations)
        
        # Update safety violations
        self.safety_violations = violations
        
        # Publish safety status
        self.publish_safety_status()
        
        # Check if emergency stop should be triggered
        if violations and not self.emergency_stop_active:
            auto_triggers = self.safety_rules.get('emergency_stop', {}).get('auto_triggers', [])
            
            for violation in violations:
                if violation['type'] in auto_triggers:
                    self.trigger_emergency_stop(violation['type'])
                    break
    
    def check_collisions(self):
        # In a real implementation, this would check for collisions between objects
        # For this minimal example, we'll just return an empty list
        return []
    
    def check_safety_zones(self):
        # In a real implementation, this would check if objects are in restricted zones
        # For this minimal example, we'll just return an empty list
        return []
    
    def trigger_emergency_stop(self, reason):
        if self.emergency_stop_active:
            return
        
        self.emergency_stop_active = True
        self.get_logger().warn(f'Emergency stop triggered: {reason}')
        
        # Publish emergency stop
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        
        # Send stop command to simulation
        cmd_msg = String()
        cmd_msg.data = 'stop'
        self.command_pub.publish(cmd_msg)
    
    def reset_emergency_stop(self):
        if not self.emergency_stop_active:
            return
        
        # Check if reset requires confirmation
        reset_requires_confirmation = self.safety_rules.get('emergency_stop', {}).get('reset_requires_confirmation', True)
        
        # Check if it's safe to reset
        if reset_requires_confirmation and self.safety_violations:
            self.get_logger().warn('Cannot reset emergency stop: safety violations still present')
            return
        
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop reset')
        
        # Publish emergency stop reset
        stop_msg = Bool()
        stop_msg.data = False
        self.emergency_stop_pub.publish(stop_msg)
    
    def publish_safety_status(self):
        status_msg = String()
        status_data = {
            'emergency_stop_active': self.emergency_stop_active,
            'safety_violations': self.safety_violations,
            'emergency_stop_enabled': self.emergency_stop_enabled,
            'collision_detection_enabled': self.collision_detection_enabled,
            'safety_zone_monitoring_enabled': self.safety_zone_monitoring_enabled
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
