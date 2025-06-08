#!/usr/bin/env python3

import os
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import json
import threading
import time
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO
import cv2
import numpy as np
import base64

SCENARIO_ID_PATTERN = re.compile(r'^[A-Za-z0-9_]+$')

from ament_index_python.packages import get_package_share_directory


class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Declare parameters using a single dictionary
        param_defaults = {
            'port': 8080,
            'host': '0.0.0.0',
            'config_dir': '',
            'data_dir': '',
            'allow_unsafe_werkzeug': True,
        }
        self.declare_parameters('', [(k, v) for k, v in param_defaults.items()])
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.config_dir = self.get_parameter('config_dir').value
        self.data_dir = self.get_parameter('data_dir').value
        self.allow_unsafe_werkzeug = self.get_parameter('allow_unsafe_werkzeug').value
        
        # Create data directory if it doesn't exist
        if self.data_dir and not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize state
        self.current_scenario = 'default'
        self.system_status = 'idle'
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_metrics = {
            'cycle_time': 0.0,
            'throughput': 0.0,
            'accuracy': 100.0,
            'errors': 0,
            'objects_processed': 0
        }
        
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
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        self.metrics_sub = self.create_subscription(
            String,
            '/simulation/metrics',
            self.metrics_callback,
            10)
        
        package_share_directory = get_package_share_directory('simulation_tools')
        template_folder_path = os.path.join(package_share_directory, 'templates')
        static_folder_path = os.path.join(package_share_directory, 'static')

        self.app = Flask(__name__,
                         template_folder=template_folder_path,
                         static_folder=static_folder_path)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Set up routes
        self.setup_routes()
        
        # Set up Socket.IO events
        self.setup_socketio_events()
        
        # Start web server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f'Web interface started at http://{self.host}:{self.port}')
    
    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.system_status = status_data.get('status', 'unknown')
            if 'current_scenario' in status_data:
                self.current_scenario = status_data['current_scenario']
            
            # Emit status update to clients
            self.socketio.emit('status_update', {
                'status': self.system_status,
                'current_scenario': self.current_scenario
            })
        except Exception as e:
            self.get_logger().error(f'Error parsing status message: {e}')
    
    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Optionally save image to file
            if self.data_dir:
                image_path = os.path.join(self.data_dir, 'latest_rgb.jpg')
                cv2.imwrite(image_path, self.latest_rgb_image)

            # Encode image and emit directly to clients
            success, buffer = cv2.imencode('.jpg', self.latest_rgb_image)
            if success:
                b64_data = base64.b64encode(buffer.tobytes()).decode('utf-8')
                self.socketio.emit('image_data', {'rgb': b64_data})
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize depth image for visualization
            depth_normalized = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Optionally save image to file
            if self.data_dir:
                image_path = os.path.join(self.data_dir, 'latest_depth.jpg')
                cv2.imwrite(image_path, depth_colormap)

            # Encode depth image and emit directly to clients
            success, buffer = cv2.imencode('.jpg', depth_colormap)
            if success:
                b64_data = base64.b64encode(buffer.tobytes()).decode('utf-8')
                self.socketio.emit('image_data', {'depth': b64_data})
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def metrics_callback(self, msg):
        try:
            metrics_data = json.loads(msg.data)
            self.latest_metrics.update(metrics_data)
            
            # Emit metrics update to clients
            self.socketio.emit('metrics_update', {
                'metrics': self.latest_metrics
            })
        except Exception as e:
            self.get_logger().error(f'Error parsing metrics message: {e}')
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        # ADD THESE TWO ROUTES
        @self.app.route('/dashboard')
        def dashboard():
            return render_template('dashboard.html')
    
        @self.app.route('/control')
        def control():
            return render_template('control.html')

        @self.app.route('/api/status')
        def get_status():
            return jsonify({
                'status': self.system_status,
                'current_scenario': self.current_scenario
            })

        @self.app.route('/api/place', methods=['POST'])
        def api_place():
            """Handle place command requests."""
            data = request.get_json(silent=True) or {}
            location = data.get('location')

            cmd = 'place'
            if location:
                cmd += f' {location}'

            cmd_msg = String()
            cmd_msg.data = cmd
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(
                f"Received place command from web UI: '{cmd}' published to /simulation/command." )

            return jsonify({"success": True, "message": "Place command sent"}), 200
        
        @self.app.route('/api/image/latest')
        def get_latest_image():
            image_type = request.args.get('type', 'rgb')
            
            if image_type == 'rgb':
                image_path = os.path.join(self.data_dir, 'latest_rgb.jpg')
            elif image_type == 'depth':
                image_path = os.path.join(self.data_dir, 'latest_depth.jpg')
            else:
                return jsonify({'error': 'Invalid image type'}), 400
            
            if os.path.exists(image_path):
                return send_from_directory(self.data_dir, os.path.basename(image_path))
            else:
                return jsonify({'error': 'Image not available'}), 404
        
        @self.app.route('/api/scenarios')
        def get_scenarios():
            scenarios = []
            
            if self.config_dir and os.path.exists(self.config_dir):
                for filename in os.listdir(self.config_dir):
                    if filename.endswith('.yaml'):
                        scenario_id = filename[:-5]  # Remove .yaml extension
                        
                        # Read description from file if available
                        description = "No description available"
                        try:
                            with open(os.path.join(self.config_dir, filename), 'r') as f:
                                config = yaml.safe_load(f)
                                if config and 'description' in config:
                                    description = config['description']
                        except Exception as e:
                            self.get_logger().error(f'Error reading scenario file {filename}: {e}')
                        
                        scenarios.append({
                            'id': scenario_id,
                            'name': scenario_id.replace('_', ' ').title(),
                            'description': description
                        })
            
            return jsonify(scenarios)

        @self.app.route('/api/scenarios', methods=['POST'])
        def upload_scenario():
            if not self.config_dir:
                return jsonify({'error': 'Config directory not set'}), 500

            scenario_id = request.form.get('scenario_id', '').strip()
            file = request.files.get('file')

            if not scenario_id or not file:
                return jsonify({'error': 'scenario_id and file are required'}), 400

            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
            try:
                file.save(scenario_path)
                return jsonify({'success': True, 'id': scenario_id})
            except Exception as e:
                self.get_logger().error(f'Error saving scenario file {scenario_id}: {e}')
                return jsonify({'error': f'Error saving scenario: {e}'}), 500
        
        @self.app.route('/api/scenarios/<scenario_id>')
        def get_scenario(scenario_id):
            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            if self.config_dir:
                scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
                
                if os.path.exists(scenario_path):
                    try:
                        with open(scenario_path, 'r') as f:
                            config = yaml.safe_load(f)
                            return jsonify({
                                'id': scenario_id,
                                'name': scenario_id.replace('_', ' ').title(),
                                'config': config
                            })
                    except Exception as e:
                        self.get_logger().error(f'Error reading scenario file {scenario_id}: {e}')
                        return jsonify({'error': f'Error reading scenario: {e}'}), 500
            
            return jsonify({'error': 'Scenario not found'}), 404

        @self.app.route('/api/scenarios/<scenario_id>', methods=['DELETE'])
        def delete_scenario(scenario_id):
            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            if not self.config_dir:
                return jsonify({'error': 'Config directory not set'}), 500

            if scenario_id == 'default':
                return jsonify({'error': 'Cannot delete default scenario'}), 400

            scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
            if os.path.exists(scenario_path):
                try:
                    os.remove(scenario_path)
                    return jsonify({'success': True})
                except Exception as e:
                    self.get_logger().error(f'Error deleting scenario file {scenario_id}: {e}')
                    return jsonify({'error': f'Error deleting scenario: {e}'}), 500

            return jsonify({'error': 'Scenario not found'}), 404
        
        @self.app.route('/static/<path:path>')
        def serve_static(path):
            return send_from_directory(self.app.static_folder, path)
    
    def setup_socketio_events(self):
        @self.socketio.on('connect')
        def handle_connect():
            self.get_logger().info('Client connected')
            
            # Send initial state
            self.socketio.emit('status_update', {
                'status': self.system_status,
                'current_scenario': self.current_scenario
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.get_logger().info('Client disconnected')
        
        @self.socketio.on('start_simulation')
        def handle_start_simulation(data):
            scenario = data.get('scenario', 'default')
            self.current_scenario = scenario
            
            # Publish command
            cmd_msg = String()
            cmd_msg.data = 'start'
            self.command_pub.publish(cmd_msg)
            
            # Publish scenario config
            config_msg = String()
            config_msg.data = json.dumps({
                'scenario': scenario
            })
            self.config_pub.publish(config_msg)
            
            self.socketio.emit('command_sent', {
                'command': 'start',
                'scenario': scenario
            })
        
        @self.socketio.on('stop_simulation')
        def handle_stop_simulation():
            # Publish command
            cmd_msg = String()
            cmd_msg.data = 'stop'
            self.command_pub.publish(cmd_msg)
            
            self.socketio.emit('command_sent', {
                'command': 'stop'
            })
        
        @self.socketio.on('send_command')
        def handle_send_command(data):
            command = data.get('command', '')
            
            # Publish command
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            
            self.socketio.emit('command_sent', {
                'command': command
            })
        
        @self.socketio.on('update_config')
        def handle_update_config(data):
            # Publish config
            config_msg = String()
            config_msg.data = json.dumps(data)
            self.config_pub.publish(config_msg)
            
            self.socketio.emit('config_updated', {
                'config': data
            })
    
    def run_server(self):
        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=self.allow_unsafe_werkzeug,
        )

def main(args=None):
    rclpy.init(args=args)
    node = WebInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
