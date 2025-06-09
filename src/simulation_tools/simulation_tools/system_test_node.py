#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import json
import threading
import time

class SystemTestNode(Node):
    def __init__(self):
        super().__init__('system_test_node')
        
        # Declare parameters
        self.declare_parameter('test_duration', 60.0)
        self.declare_parameter('test_scenario', 'default')
        self.declare_parameter('data_dir', '')
        
        # Get parameters
        self.test_duration = self.get_parameter('test_duration').value
        self.test_scenario = self.get_parameter('test_scenario').value
        self.data_dir = self.get_parameter('data_dir').value
        
        # Initialize state
        self.test_running = False
        self.test_results = {}
        self.start_time = 0.0
        
        # Create publishers
        self.command_pub = self.create_publisher(
            String, 
            '/simulation/command', 
            10)
        self.config_pub = self.create_publisher(
            String, 
            '/simulation/config', 
            10)
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            String,
            '/simulation/status',
            self.status_callback,
            10)
        self.metrics_sub = self.create_subscription(
            String,
            '/simulation/metrics',
            self.metrics_callback,
            10)
        self.safety_sub = self.create_subscription(
            String,
            '/safety/status',
            self.safety_callback,
            10)
        
        # Create timer for test monitoring
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('System test node initialized')
    
    def start_test(self):
        if self.test_running:
            self.get_logger().warning('Test already running')
            return
        
        self.test_running = True
        self.start_time = time.time()
        self.test_results = {
            'scenario': self.test_scenario,
            'start_time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.start_time)),
            'duration': self.test_duration,
            'metrics': {},
            'safety_events': [],
            'errors': []
        }
        
        self.get_logger().info(f'Starting system test with scenario {self.test_scenario} for {self.test_duration} seconds')
        
        # Send start command with scenario
        config_msg = String()
        config_msg.data = json.dumps({
            'scenario': self.test_scenario
        })
        self.config_pub.publish(config_msg)
        
        # Send start command
        cmd_msg = String()
        cmd_msg.data = 'start'
        self.command_pub.publish(cmd_msg)
    
    def stop_test(self):
        if not self.test_running:
            return
        
        self.test_running = False
        end_time = time.time()
        elapsed = end_time - self.start_time
        
        self.test_results['end_time'] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(end_time))
        self.test_results['actual_duration'] = elapsed
        
        self.get_logger().info(f'Test completed in {elapsed:.2f} seconds')
        
        # Send stop command
        cmd_msg = String()
        cmd_msg.data = 'stop'
        self.command_pub.publish(cmd_msg)
        
        # Save test results
        if self.data_dir:
            results_path = os.path.join(self.data_dir, f'test_results_{time.strftime("%Y%m%d_%H%M%S")}.json')
            try:
                with open(results_path, 'w') as f:
                    json.dump(self.test_results, f, indent=2)
                self.get_logger().info(f'Test results saved to {results_path}')
            except Exception as e:
                self.get_logger().error(f'Error saving test results: {e}')
    
    def status_callback(self, msg):
        if not self.test_running:
            return
        
        try:
            status_data = json.loads(msg.data)
            self.test_results['status'] = status_data
        except Exception as e:
            self.get_logger().error(f'Error parsing status message: {e}')
            self.test_results['errors'].append({
                'time': time.time() - self.start_time,
                'type': 'status_parse_error',
                'message': str(e)
            })
    
    def metrics_callback(self, msg):
        if not self.test_running:
            return
        
        try:
            metrics_data = json.loads(msg.data)
            self.test_results['metrics'] = metrics_data
        except Exception as e:
            self.get_logger().error(f'Error parsing metrics message: {e}')
            self.test_results['errors'].append({
                'time': time.time() - self.start_time,
                'type': 'metrics_parse_error',
                'message': str(e)
            })
    
    def safety_callback(self, msg):
        if not self.test_running:
            return
        
        try:
            safety_data = json.loads(msg.data)
            
            # Check for safety violations
            if safety_data.get('emergency_stop_active', False):
                self.test_results['safety_events'].append({
                    'time': time.time() - self.start_time,
                    'type': 'emergency_stop',
                    'data': safety_data
                })
            
            if safety_data.get('safety_violations', []):
                self.test_results['safety_events'].append({
                    'time': time.time() - self.start_time,
                    'type': 'safety_violation',
                    'violations': safety_data['safety_violations']
                })
        except Exception as e:
            self.get_logger().error(f'Error parsing safety message: {e}')
            self.test_results['errors'].append({
                'time': time.time() - self.start_time,
                'type': 'safety_parse_error',
                'message': str(e)
            })
    
    def timer_callback(self):
        if not self.test_running:
            return
        
        elapsed = time.time() - self.start_time
        
        # Check if test duration has elapsed
        if elapsed >= self.test_duration:
            self.get_logger().info('Test duration reached, stopping test')
            self.stop_test()
            return
        
        # Log progress
        if int(elapsed) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f'Test running for {elapsed:.1f} seconds, {self.test_duration - elapsed:.1f} seconds remaining')

def main(args=None):
    rclpy.init(args=args)
    node = SystemTestNode()
    
    try:
        # Start test
        node.start_test()
        
        # Spin until test completes
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
        node.stop_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
