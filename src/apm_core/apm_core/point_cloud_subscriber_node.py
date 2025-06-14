#!/usr/bin/env python3
"""ROS2 node subscribing to a point cloud topic and printing statistics."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class PointCloudSubscriberNode(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber_node')
        
        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        
        # Create subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10)
        
        self.get_logger().info(f'Point cloud subscriber initialized. Listening on topic: {pointcloud_topic}')
    
    def pointcloud_callback(self, msg):
        try:
            # Log point cloud information
            self.get_logger().info(f'Received point cloud: {msg.width}x{msg.height}, {msg.point_step} bytes per point')
            
            # Process point cloud here
            # Example: Extract a few points for demonstration
            points = self.extract_points(msg, 5)  # Extract 5 points as example
            for i, point in enumerate(points):
                self.get_logger().info(f'Point {i}: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def extract_points(self, cloud_msg, num_points=5):
        """Extract a specified number of points from the point cloud message."""
        points = []
        
        # Get cloud data
        cloud_data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(-1, cloud_msg.point_step)
        
        # Extract points (assuming XYZ format)
        step = len(cloud_data) // num_points
        if step == 0:
            step = 1
        
        for i in range(0, len(cloud_data), step):
            if len(points) >= num_points:
                break
                
            # Extract XYZ coordinates (assuming float32 format)
            x = struct.unpack_from('f', cloud_data[i], 0)[0]
            y = struct.unpack_from('f', cloud_data[i], 4)[0]
            z = struct.unpack_from('f', cloud_data[i], 8)[0]
            
            # Skip invalid points
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append((x, y, z))
        
        return points

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
