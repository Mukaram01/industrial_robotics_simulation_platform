#!/usr/bin/env python3

"""
Web-based interface node for controlling the simulation.

This example follows the standard robot integration interfaces described in docs/robot_integration_guide.md. Commands are published on `/simulation/command` (which can be remapped to `/robot/command`) and status messages are read from `/simulation/status`.
"""

import os
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from typing import Optional, Any, cast
import yaml  # type: ignore
import json
import threading
import time
import shutil
import webbrowser
from flask import Flask, render_template, request, jsonify, send_from_directory, redirect
from flask_socketio import SocketIO
from flask_login import (
    LoginManager,
    UserMixin,
    login_user,
    logout_user,
    login_required,
)
import cv2
import numpy as np
import base64

SCENARIO_ID_PATTERN = re.compile(r'^[A-Za-z0-9_]+$')

from ament_index_python.packages import get_package_share_directory
from .action_logger import ActionLogger

try:
    from apm_msgs.msg import DetectedObjectArray
except Exception:  # pragma: no cover - optional dependency
    DetectedObjectArray = None


class SimpleUser(UserMixin):
    """Minimal user object for Flask-Login."""

    def __init__(self, user_id: str) -> None:
        self.id = user_id


class WebInterfaceNode(Node):
    """Exposes a simple Flask web UI for interacting with the simulator."""

    def __init__(self) -> None:
        """Initialize the web server, ROS interfaces and parameters."""
        super().__init__('web_interface_node')
        
        # Declare parameters using a single dictionary
        param_defaults = {
            'port': 8080,
            'host': '0.0.0.0',
            'config_dir': '',
            'data_dir': '',
            'save_images': False,
            'allow_unsafe_werkzeug': True,
            'log_db_path': '',
            'jpeg_quality': 75,
            'detected_objects_topic': '/apm/detection/objects',
            'joint_states_topic': '/joint_states',
            'auto_open_browser': False,
        }
        self.declare_parameters('', [(k, v) for k, v in param_defaults.items()])
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.config_dir = self.get_parameter('config_dir').value
        self.data_dir = self.get_parameter('data_dir').value
        self.save_images = self.get_parameter('save_images').value
        self.allow_unsafe_werkzeug = self.get_parameter('allow_unsafe_werkzeug').value
        self.log_db_path = self.get_parameter('log_db_path').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not 0 <= self.jpeg_quality <= 100:
            self.get_logger().warning(
                f'Invalid jpeg_quality {self.jpeg_quality}, using 75 instead'
            )
            self.jpeg_quality = 75
        self.detected_objects_topic = self.get_parameter('detected_objects_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.auto_open_browser = self.get_parameter('auto_open_browser').value

        if not self.log_db_path:
            if self.data_dir:
                self.log_db_path = os.path.join(self.data_dir, 'actions.db')
            else:
                self.log_db_path = os.path.join(os.getcwd(), 'actions.db')

        # Ensure data directory exists before the logger uses it
        if self.data_dir and not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        self.action_logger = ActionLogger(self.log_db_path)

        # Load user credentials
        self.users = self._load_users()
            
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize state
        self.current_scenario = 'default'
        self.system_status = 'idle'
        self.latest_rgb_image: Optional[np.ndarray] = None
        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_metrics = {
            'cycle_time': 0.0,
            'throughput': 0.0,
            'accuracy': 100.0,
            'errors': 0,
            'objects_processed': 0
        }
        self.latest_objects: list[dict[str, Any]] = []
        self.latest_joint_state: dict[str, list[Any]] | None = None
        
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

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10)

        # Subscribe to detected objects if message type is available
        if DetectedObjectArray is not None:
            self.objects_sub = self.create_subscription(
                DetectedObjectArray,
                self.detected_objects_topic,
                self.objects_callback,
                10)
        else:
            self.get_logger().warning(
                'apm_msgs not available, object updates disabled')
        
        package_share_directory = get_package_share_directory('web_interface_frontend')
        template_folder_path = os.path.join(package_share_directory, 'templates')
        static_folder_path = os.path.join(package_share_directory, 'static')

        self.app = Flask(__name__,
                         template_folder=template_folder_path,
                         static_folder=static_folder_path)
        self.app.secret_key = 'secret'

        self.login_manager = LoginManager()
        self.login_manager.login_view = 'login'
        self.login_manager.init_app(self.app)

        @self.login_manager.user_loader
        def load_user(user_id: str) -> SimpleUser | None:  # pragma: no cover - simple loader
            if user_id in self.users:
                return SimpleUser(user_id)
            return None

        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Set up routes
        self.setup_routes()
        
        # Set up Socket.IO events
        self.setup_socketio_events()
        
        # Start web server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

        if self.auto_open_browser:
            url = f'http://{self.host}:{self.port}'
            threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()
        
        self.get_logger().info(f'Web interface started at http://{self.host}:{self.port}')
    
    def status_callback(self, msg: String) -> None:
        """Update internal status when the simulator publishes a status message."""
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
            if self.latest_joint_state is not None:
                self.socketio.emit('joint_state_update', self.latest_joint_state)
        except Exception as e:
            self.get_logger().error(f'Error parsing status message: {e}')
    
    def rgb_callback(self, msg: Image) -> None:
        """Handle incoming RGB images and forward them to web clients."""
        try:
            img = cast(np.ndarray, self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            self.latest_rgb_image = img

            assert self.latest_rgb_image is not None
            success, buffer = cv2.imencode(
                '.jpg',
                self.latest_rgb_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )

            if self.save_images and self.data_dir and success:
                image_path = os.path.join(self.data_dir, 'latest_rgb.jpg')
                with open(image_path, 'wb') as f:
                    f.write(buffer.tobytes())


            # Encode image and emit directly to clients
            if success:
                b64_data = base64.b64encode(buffer.tobytes()).decode('utf-8')
                self.socketio.emit('image_data', {'rgb': b64_data})
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')
    
    def depth_callback(self, msg: Image) -> None:
        """Handle incoming depth images and forward them to web clients."""
        try:
            depth_img = cast(np.ndarray, self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
            self.latest_depth_image = depth_img

            assert self.latest_depth_image is not None
            # Normalize depth image for visualization
            depth_normalized = np.empty(depth_img.shape, dtype=np.uint8)
            cv2.normalize(self.latest_depth_image, depth_normalized, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            success, buffer = cv2.imencode(
                '.jpg',
                depth_colormap,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )

            # Optionally save image to file

            if self.save_images and self.data_dir and success:
                image_path = os.path.join(self.data_dir, 'latest_depth.jpg')
                with open(image_path, 'wb') as f:
                    f.write(buffer.tobytes())


            # Encode depth image and emit directly to clients
            if success:
                b64_data = base64.b64encode(buffer.tobytes()).decode('utf-8')
                self.socketio.emit('image_data', {'depth': b64_data})
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def metrics_callback(self, msg: String) -> None:
        """Receive system metrics and broadcast them to clients."""
        try:
            metrics_data = json.loads(msg.data)
            self.latest_metrics.update(metrics_data)
            
            # Emit metrics update to clients
            self.socketio.emit('metrics_update', {
                'metrics': self.latest_metrics
            })
        except Exception as e:
            self.get_logger().error(f'Error parsing metrics message: {e}')

    def objects_callback(self, msg: Any) -> None:
        """Forward detected object information to web clients."""
        try:
            objects = []
            for obj in msg.objects:
                objects.append({
                    'id': obj.id,
                    'class': getattr(obj, 'class_name', str(obj.class_id)),
                    'score': getattr(obj, 'confidence', 0.0),
                    'position': {
                        'x': obj.pose.position.x,
                        'y': obj.pose.position.y,
                        'z': obj.pose.position.z,
                    },
                    'size': {
                        'x': obj.dimensions.x,
                        'y': obj.dimensions.y,
                        'z': obj.dimensions.z,
                    },
                })
            self.latest_objects = objects
            self.socketio.emit('objects_update', {'objects': objects})
        except Exception as e:
            self.get_logger().error(f'Error processing detected objects: {e}')

    def joint_state_callback(self, msg: JointState) -> None:
        try:
            js_data = {
                'name': list(msg.name),
                'position': list(msg.position),
                'velocity': list(msg.velocity),
            }
            self.latest_joint_state = js_data
            self.socketio.emit('joint_state_update', js_data)
        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')

    def _load_users(self) -> dict[str, str]:
        """Load users from a YAML file if available."""
        if self.config_dir:
            path = os.path.join(self.config_dir, 'users.yaml')
            if os.path.exists(path):
                try:
                    data = yaml.safe_load(open(path, 'r'))
                    if isinstance(data, dict):
                        return {str(k): str(v) for k, v in data.items()}
                except Exception as e:  # pragma: no cover - log and fall back
                    self.get_logger().error(f'Error loading users: {e}')
        return {'admin': 'admin'}

    def setup_routes(self) -> None:
        """Register Flask HTTP routes for the user interface."""
        @self.app.route('/login', methods=['GET', 'POST'])
        def login():
            if request.method == 'POST':
                username = request.form.get('username', '')
                password = request.form.get('password', '')
                if self.users.get(username) == password:
                    login_user(SimpleUser(username))
                    return redirect(request.args.get('next') or '/')
                return render_template('login.html', error='Invalid credentials')
            return render_template('login.html')

        @self.app.route('/logout')
        @login_required
        def logout():
            logout_user()
            return redirect('/login')

        @self.app.route('/')
        @login_required
        def index():
            return render_template('index.html')

        @self.app.route('/dashboard')
        @login_required
        def dashboard():
            return render_template('dashboard.html')

        @self.app.route('/control')
        @login_required
        def control():
            return render_template('control.html')

        @self.app.route('/log')
        @login_required
        def log_page():
            return render_template('log.html')

        @self.app.route('/editor')
        @login_required
        def editor():
            return render_template('editor.html')

        @self.app.route('/api/status')
        @login_required
        def get_status():
            return jsonify({
                'status': self.system_status,
                'current_scenario': self.current_scenario
            })

        @self.app.route('/api/metrics')
        @login_required
        def get_metrics():
            return jsonify({'metrics': self.latest_metrics})

        @self.app.route('/api/place', methods=['POST'])
        @login_required
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
            self.action_logger.log('place', {'location': location})

            return jsonify({"success": True, "message": "Place command sent"}), 200

        @self.app.route('/api/pick', methods=['POST'])
        @login_required
        def api_pick():
            """Handle pick command requests."""
            data = request.get_json(silent=True) or {}
            object_id = data.get('object_id')
            if not object_id:
                return jsonify({'error': 'object_id required'}), 400

            cmd_msg = String()
            cmd_msg.data = f'pick {object_id}'
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(
                f"Received pick command from web UI: 'pick {object_id}' published to /simulation/command.")
            self.action_logger.log('pick', {'object_id': object_id})

            return jsonify({'success': True, 'message': 'Pick command sent'}), 200
        
        @self.app.route('/api/image/latest')
        @login_required
        def get_latest_image():
            image_type = request.args.get('type', 'rgb')
            if image_type not in ('rgb', 'depth'):
                return jsonify({'error': 'Invalid image type'}), 400

            if self.save_images and self.data_dir:
                image_path = os.path.join(self.data_dir, f'latest_{image_type}.jpg')
                if os.path.exists(image_path):
                    return send_from_directory(self.data_dir, os.path.basename(image_path))

            # Fallback to in-memory image encoded as base64
            image = None
            if image_type == 'rgb':
                image = self.latest_rgb_image
            else:
                if self.latest_depth_image is not None:
                    depth_normalized = cv2.normalize(self.latest_depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            if image is not None:
                success, buffer = cv2.imencode('.jpg', image)
                if success:
                    b64_data = base64.b64encode(buffer.tobytes()).decode('utf-8')
                    return jsonify({'base64': b64_data})

            return jsonify({'error': 'Image not available'}), 404

        @self.app.route('/api/objects')
        @login_required
        def get_objects():
            return jsonify({'objects': self.latest_objects})
        
        @self.app.route('/api/scenarios')
        @login_required
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
        @login_required
        def upload_scenario():
            if not self.config_dir:
                return jsonify({'error': 'Config directory not set'}), 500

            scenario_id = request.form.get('scenario_id', '').strip()
            file = request.files.get('file')

            if not scenario_id or not file:
                return jsonify({'error': 'scenario_id and file are required'}), 400

            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            if not file.filename.lower().endswith('.yaml'):
                return jsonify({'error': 'Only .yaml files allowed'}), 400

            scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')

            if os.path.exists(scenario_path):
                return jsonify({'error': 'Scenario already exists'}), 409

            try:
                data = file.read()
                try:
                    yaml.safe_load(data or '')
                except Exception as e:
                    self.get_logger().error(
                        f'Invalid YAML uploaded for scenario {scenario_id}: {e}'
                    )
                    return jsonify({'error': 'Invalid YAML file'}), 400

                with open(scenario_path, 'wb') as f:
                    f.write(data)
                self.action_logger.log('upload_scenario', {'id': scenario_id})
                return jsonify({'success': True, 'id': scenario_id})
            except Exception as e:
                self.get_logger().error(
                    f'Error saving scenario file {scenario_id}: {e}'
                )
                return jsonify({'error': f'Error saving scenario: {e}'}), 500
        
        @self.app.route('/api/scenarios/<scenario_id>')
        @login_required
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

        @self.app.route('/api/scenarios/<scenario_id>/load', methods=['POST'])
        @login_required
        def load_scenario_api(scenario_id):
            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            msg = String()
            msg.data = json.dumps({'scenario': scenario_id})
            self.config_pub.publish(msg)
            self.action_logger.log('load_scenario', {'id': scenario_id})
            return jsonify({'success': True})

        @self.app.route('/api/scenarios/<scenario_id>', methods=['PUT'])
        @login_required
        def save_scenario_api(scenario_id):
            if not SCENARIO_ID_PATTERN.match(scenario_id):
                return jsonify({'error': 'Invalid scenario ID'}), 400

            data = request.get_json(silent=True) or {}
            config = data.get('config')
            if config is None:
                return jsonify({'error': 'config required'}), 400

            if self.config_dir:
                scenario_path = os.path.join(self.config_dir, f'{scenario_id}.yaml')
                try:
                    with open(scenario_path, 'w') as f:
                        yaml.safe_dump(config, f, default_flow_style=False)
                except Exception as e:
                    self.get_logger().error(f'Error saving scenario file {scenario_id}: {e}')
                    return jsonify({'error': f'Error saving scenario: {e}'}), 500

            msg = String()
            msg.data = json.dumps({'update_scenario': {'name': scenario_id, 'config': config}})
            self.config_pub.publish(msg)
            self.action_logger.log('save_scenario', {'id': scenario_id})
            return jsonify({'success': True})

        @self.app.route('/api/scenarios/<scenario_id>', methods=['DELETE'])
        @login_required
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
                    self.action_logger.log('delete_scenario', {'id': scenario_id})
                    return jsonify({'success': True})
                except Exception as e:
                    self.get_logger().error(f'Error deleting scenario file {scenario_id}: {e}')
                    return jsonify({'error': f'Error deleting scenario: {e}'}), 500

            return jsonify({'error': 'Scenario not found'}), 404

        @self.app.route('/api/command', methods=['POST'])
        @login_required
        def api_command():
            data = request.get_json(silent=True) or {}
            command = data.get('command')
            if not command:
                return jsonify({'error': 'command required'}), 400

            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('command', {'command': command})
            return jsonify({'success': True})

        @self.app.route('/api/jog', methods=['POST'])
        @login_required
        def api_jog():
            data = request.get_json(silent=True) or {}
            joint = data.get('joint')
            delta = data.get('delta')
            if joint is None or delta is None:
                return jsonify({'error': 'joint and delta required'}), 400

            cmd_msg = String()
            cmd_msg.data = f'jog {joint} {delta}'
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('jog', {'joint': joint, 'delta': delta})
            return jsonify({'success': True})

        @self.app.route('/api/waypoint', methods=['POST'])
        @login_required
        def api_waypoint():
            data = request.get_json(silent=True) or {}
            action = data.get('action')
            mapping = {
                'record': 'record_waypoint',
                'clear': 'clear_waypoints',
                'execute': 'execute_sequence',
            }
            if action not in mapping:
                return jsonify({'error': 'invalid action'}), 400

            cmd_msg = String()
            cmd_msg.data = mapping[action]
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('waypoint', {'action': action})
            return jsonify({'success': True})

        @self.app.route('/api/actions')
        @login_required
        def api_actions():
            limit = int(request.args.get('limit', 100))
            rows = []
            try:
                rows = self.action_logger.get_recent_actions(limit)
            except Exception as e:
                self.get_logger().error(f'Error reading actions: {e}')
            return jsonify({'actions': rows})

        @self.app.route('/api/exports')
        @login_required
        def api_exports():
            exports_dir = os.path.join(self.data_dir, 'exports') if self.data_dir else ''
            result = []
            if exports_dir and os.path.exists(exports_dir):
                for name in sorted(os.listdir(exports_dir)):
                    path = os.path.join(exports_dir, name)
                    if os.path.isdir(path):
                        result.append({'name': name})
            return jsonify({'exports': result})

        @self.app.route('/api/exports/<export_id>')
        @login_required
        def download_export(export_id):
            exports_dir = os.path.join(self.data_dir, 'exports') if self.data_dir else ''
            path = os.path.join(exports_dir, export_id)
            if not exports_dir or not os.path.isdir(path):
                return jsonify({'error': 'Export not found'}), 404
            archive = shutil.make_archive('/tmp/' + export_id, 'zip', path)
            return send_from_directory('/tmp', os.path.basename(archive), as_attachment=True)
        
        @self.app.route('/static/<path:path>')
        def serve_static(path):
            return send_from_directory(self.app.static_folder, path)
    
    def setup_socketio_events(self) -> None:
        """Register Socket.IO event handlers for real-time control."""
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

            self.action_logger.log('start_simulation', {'scenario': scenario})
            
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

            self.action_logger.log('stop_simulation')
            
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

            self.action_logger.log('send_command', {'command': command})
            
            self.socketio.emit('command_sent', {
                'command': command
            })
        
        @self.socketio.on('update_config')
        def handle_update_config(data):
            # Publish config
            config_msg = String()
            config_msg.data = json.dumps(data)
            self.config_pub.publish(config_msg)

            self.action_logger.log('update_config', data)

            self.socketio.emit('config_updated', {
                'config': data
            })

        @self.socketio.on('jog_joint')
        def handle_jog_joint(data):
            joint = data.get('joint')
            delta = data.get('delta')
            if joint is None or delta is None:
                return
            cmd_msg = String()
            cmd_msg.data = f'jog {joint} {delta}'
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('jog', {'joint': joint, 'delta': delta})
            self.socketio.emit('command_sent', {'command': cmd_msg.data})

        @self.socketio.on('record_waypoint')
        def handle_record_waypoint():
            cmd_msg = String()
            cmd_msg.data = 'record_waypoint'
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('waypoint', {'action': 'record'})
            self.socketio.emit('command_sent', {'command': 'record_waypoint'})

        @self.socketio.on('execute_sequence')
        def handle_execute_sequence():
            cmd_msg = String()
            cmd_msg.data = 'execute_sequence'
            self.command_pub.publish(cmd_msg)
            self.action_logger.log('waypoint', {'action': 'execute'})
            self.socketio.emit('command_sent', {'command': 'execute_sequence'})

        @self.socketio.on('load_scenario')
        def handle_load_scenario_socket(data):
            scenario = data.get('scenario')
            if not scenario:
                return
            msg = String()
            msg.data = json.dumps({'scenario': scenario})
            self.config_pub.publish(msg)
            self.action_logger.log('load_scenario', {'id': scenario})
            self.socketio.emit('scenario_loaded', {'id': scenario})

        @self.socketio.on('save_scenario')
        def handle_save_scenario_socket(data):
            name = data.get('name')
            config = data.get('config')
            if not name or config is None:
                return
            if self.config_dir:
                path = os.path.join(self.config_dir, f'{name}.yaml')
                try:
                    with open(path, 'w') as f:
                        yaml.safe_dump(config, f, default_flow_style=False)
                except Exception:
                    self.socketio.emit('scenario_saved', {'success': False})
                    return
            msg = String()
            msg.data = json.dumps({'update_scenario': {'name': name, 'config': config}})
            self.config_pub.publish(msg)
            self.action_logger.log('save_scenario', {'id': name})
            self.socketio.emit('scenario_saved', {'success': True, 'id': name})

    def run_server(self) -> None:
        """Launch the Flask-SocketIO server."""
        try:
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=self.allow_unsafe_werkzeug,
            )
        except TypeError:
            self.get_logger().warning(
                'SocketIO.run() does not accept allow_unsafe_werkzeug, retrying'
            )
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                debug=False,
                use_reloader=False,
            )

    def shutdown(self) -> None:
        """Stop the Socket.IO server and wait for its thread to finish."""
        try:
            self.socketio.stop()
        except Exception as e:  # pragma: no cover - best effort shutdown
            self.get_logger().error(f'Error stopping SocketIO server: {e}')
        # Ensure the server thread is joined to avoid dangling threads
        if self.server_thread.is_alive():
            self.server_thread.join()

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.action_logger.close()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
