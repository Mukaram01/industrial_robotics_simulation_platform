#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from apm_msgs.msg import DetectedObjectArray
import threading
import json
import time

class ROS2Bridge:
    """
    Bridge between ROS2 and Flask application
    """
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        
        # Create node
        self.node = Node('web_interface_node')
        
        # Create subscribers
        self.status_sub = self.node.create_subscription(
            String,
            '/fmm/status',
            self.status_callback,
            10
        )
        
        self.objects_sub = self.node.create_subscription(
            DetectedObjectArray,
            '/apm/advanced_perception/objects',
            self.objects_callback,
            10
        )
        
        # Create publishers
        self.pick_command_pub = self.node.create_publisher(
            String,
            '/fmm/pick_command',
            10
        )
        
        self.place_command_pub = self.node.create_publisher(
            String,
            '/fmm/place_command',
            10
        )
        
        # Initialize state
        self.status = 'unknown'
        self.detected_objects = []
        self.last_objects_update = 0
        
        # Create ROS2 spin thread
        self.spin_thread = threading.Thread(target=self._spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
        self.node.get_logger().info('ROS2 Bridge initialized')
    
    def _spin(self):
        """
        Spin ROS2 node in a separate thread
        """
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"Error in ROS2 spin thread: {str(e)}")
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
    
    def status_callback(self, msg):
        """
        Callback for robot status messages
        """
        self.status = msg.data
        self.node.get_logger().debug(f"Received status: {self.status}")
    
    def objects_callback(self, msg):
        """
        Callback for detected objects messages
        """
        # Convert detected objects to a list of dictionaries
        objects = []
        for obj in msg.objects:
            objects.append({
                'id': obj.id,
                'class': obj.class_name,
                'score': obj.score,
                'position': {
                    'x': obj.pose.pose.position.x,
                    'y': obj.pose.pose.position.y,
                    'z': obj.pose.pose.position.z
                },
                'size': {
                    'x': obj.size.x,
                    'y': obj.size.y,
                    'z': obj.size.z
                }
            })
        
        self.detected_objects = objects
        self.last_objects_update = time.time()
        self.node.get_logger().debug(f"Received {len(objects)} objects")
    
    def get_status(self):
        """
        Get current robot status
        """
        return self.status
    
    def get_detected_objects(self):
        """
        Get list of detected objects
        """
        return self.detected_objects
    
    def send_pick_command(self, object_id):
        """
        Send pick command for specified object
        """
        msg = String()
        msg.data = f"pick {object_id}"
        self.pick_command_pub.publish(msg)
        self.node.get_logger().info(f"Sent pick command for object {object_id}")
        return True
    
    def send_place_command(self, location):
        """
        Send place command for specified location
        """
        msg = String()
        msg.data = f"place {location}"
        self.place_command_pub.publish(msg)
        self.node.get_logger().info(f"Sent place command for location {location}")
        return True
    
    def shutdown(self):
        """
        Shutdown ROS2 node
        """
        self.node.destroy_node()
        rclpy.shutdown()

# Singleton instance
_bridge = None

def get_bridge():
    """
    Get singleton instance of ROS2Bridge
    """
    global _bridge
    if _bridge is None:
        _bridge = ROS2Bridge()
    return _bridge
