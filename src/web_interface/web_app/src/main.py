#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from flask import Flask, render_template, jsonify, request, redirect, url_for
from flask_socketio import SocketIO, emit
import threading
import time
import json
from src.ros2_bridge import get_bridge

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode='threading')

# Get ROS2 bridge
ros2_bridge = get_bridge()

# Background thread for status updates
def background_thread():
    """
    Background thread for sending status updates to clients
    """
    last_status = None
    last_objects = None
    
    while True:
        # Get current status
        status = ros2_bridge.get_status()
        
        # Send status update if changed
        if status != last_status:
            socketio.emit('status_update', {'status': status})
            last_status = status
        
        # Get detected objects
        objects = ros2_bridge.get_detected_objects()
        
        # Send objects update if changed
        if objects != last_objects:
            socketio.emit('objects_update', {'objects': objects})
            last_objects = objects
        
        # Sleep for a short time
        socketio.sleep(0.1)

# Start background thread
thread = None
@socketio.on('connect')
def connect():
    global thread
    if thread is None:
        thread = socketio.start_background_task(background_thread)
    emit('status_update', {'status': ros2_bridge.get_status()})
    emit('objects_update', {'objects': ros2_bridge.get_detected_objects()})

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/dashboard')
def dashboard():
    return render_template('dashboard.html')

@app.route('/control')
def control():
    return render_template('control.html')

# API routes
@app.route('/api/status')
def api_status():
    return jsonify({'status': ros2_bridge.get_status()})

@app.route('/api/objects')
def api_objects():
    return jsonify({'objects': ros2_bridge.get_detected_objects()})

@app.route('/api/pick', methods=['POST'])
def api_pick():
    data = request.json
    object_id = data.get('object_id')
    
    if object_id is None:
        return jsonify({'success': False, 'error': 'Missing object_id'}), 400
    
    success = ros2_bridge.send_pick_command(object_id)
    return jsonify({'success': success})

@app.route('/api/place', methods=['POST'])
def api_place():
    data = request.json
    location = data.get('location')
    
    if location is None:
        return jsonify({'success': False, 'error': 'Missing location'}), 400
    
    success = ros2_bridge.send_place_command(location)
    return jsonify({'success': success})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)
