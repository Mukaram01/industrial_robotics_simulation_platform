#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time

class VisualizationServerNode(Node):
    def __init__(self):
        super().__init__('visualization_server_node')
        
        # Declare parameters
        self.declare_parameter('data_dir', '')
        self.declare_parameter('export_enabled', True)
        self.declare_parameter('export_interval', 60.0)
        self.declare_parameter('visualization_rate', 10.0)
        self.declare_parameter('jpeg_quality', 75)
        
        # Get parameters
        self.data_dir = self.get_parameter('data_dir').value
        self.export_enabled = self.get_parameter('export_enabled').value
        self.export_interval = self.get_parameter('export_interval').value
        self.visualization_rate = self.get_parameter('visualization_rate').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        
        # Create data directory if it doesn't exist
        if self.data_dir and not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            
        # Create exports directory if enabled
        if self.export_enabled and self.data_dir:
            self.exports_dir = os.path.join(self.data_dir, 'exports')
            if not os.path.exists(self.exports_dir):
                os.makedirs(self.exports_dir)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize state
        self.rgb_image = None
        self.depth_image = None
        self.metrics = {}
        self.environment_config = {}
        self.last_export_time = time.time()
        
        # Create publishers
        self.combined_view_pub = self.create_publisher(
            Image, 
            '/visualization/combined_view', 
            10)
        self.metrics_view_pub = self.create_publisher(
            Image, 
            '/visualization/metrics', 
            10)
        
        # Create subscribers
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
        self.status_sub = self.create_subscription(
            String,
            '/simulation/status',
            self.status_callback,
            10)
        self.config_sub = self.create_subscription(
            String,
            '/simulation/config',
            self.config_callback,
            10)
        
        # Start visualization timer
        self.visualization_timer = self.create_timer(
            1.0 / self.visualization_rate, 
            self.visualization_callback)
        
        # Start export timer if enabled
        if self.export_enabled:
            self.export_timer = self.create_timer(
                self.export_interval, 
                self.export_callback)
        
        self.get_logger().info('Visualization server node initialized')
    
    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def metrics_callback(self, msg):
        try:
            self.metrics = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Error parsing metrics message: {e}')
    
    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.system_status = status_data.get('status', 'unknown')
            if 'current_scenario' in status_data:
                self.current_scenario = status_data['current_scenario']
        except Exception as e:
            self.get_logger().error(f'Error parsing status message: {e}')
    
    def config_callback(self, msg):
        try:
            config_data = json.loads(msg.data)
            if 'environment' in config_data:
                self.environment_config = config_data['environment']
        except Exception as e:
            self.get_logger().error(f'Error parsing config message: {e}')
    
    def visualization_callback(self):
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # Create combined view
        combined_view = self.create_combined_view()
        if combined_view is not None:
            # Publish combined view
            combined_msg = self.bridge.cv2_to_imgmsg(combined_view, encoding='bgr8')
            combined_msg.header.stamp = self.get_clock().now().to_msg()
            combined_msg.header.frame_id = 'camera_color_optical_frame'
            self.combined_view_pub.publish(combined_msg)
            
            # Save combined view
            if self.data_dir:
                success, buffer = cv2.imencode(
                    '.jpg',
                    combined_view,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if success:
                    with open(
                        os.path.join(self.data_dir, 'combined_view.jpg'), 'wb'
                    ) as f:
                        f.write(buffer.tobytes())
        
        # Create metrics visualization
        metrics_view = self.create_metrics_visualization()
        if metrics_view is not None:
            # Publish metrics view
            metrics_msg = self.bridge.cv2_to_imgmsg(metrics_view, encoding='bgr8')
            metrics_msg.header.stamp = self.get_clock().now().to_msg()
            metrics_msg.header.frame_id = 'metrics_view'
            self.metrics_view_pub.publish(metrics_msg)
            
            # Save metrics view
            if self.data_dir:
                success, buffer = cv2.imencode(
                    '.jpg',
                    metrics_view,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if success:
                    with open(
                        os.path.join(self.data_dir, 'metrics_view.jpg'), 'wb'
                    ) as f:
                        f.write(buffer.tobytes())
    
    def export_callback(self):
        if not self.export_enabled or not self.data_dir:
            return
        
        current_time = time.time()
        if current_time - self.last_export_time < self.export_interval:
            return
        
        self.last_export_time = current_time
        
        # Export data
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        export_path = os.path.join(self.exports_dir, f'export_{timestamp}')
        os.makedirs(export_path, exist_ok=True)
        
        # Export images
        if self.rgb_image is not None:
            success, buffer = cv2.imencode(
                '.jpg',
                self.rgb_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if success:
                with open(os.path.join(export_path, 'rgb.jpg'), 'wb') as f:
                    f.write(buffer.tobytes())
        
        if self.depth_image is not None:
            # Normalize depth for visualization
            depth_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            success, buffer = cv2.imencode(
                '.jpg',
                depth_colormap,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if success:
                with open(os.path.join(export_path, 'depth.jpg'), 'wb') as f:
                    f.write(buffer.tobytes())
        
        # Export combined view
        combined_view = self.create_combined_view()
        if combined_view is not None:
            success, buffer = cv2.imencode(
                '.jpg',
                combined_view,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if success:
                with open(
                    os.path.join(export_path, 'combined_view.jpg'), 'wb'
                ) as f:
                    f.write(buffer.tobytes())
        
        # Export metrics visualization
        metrics_view = self.create_metrics_visualization()
        if metrics_view is not None:
            success, buffer = cv2.imencode(
                '.jpg',
                metrics_view,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if success:
                with open(
                    os.path.join(export_path, 'metrics_view.jpg'), 'wb'
                ) as f:
                    f.write(buffer.tobytes())
        
        # Export metrics data
        if self.metrics:
            with open(os.path.join(export_path, 'metrics.json'), 'w') as f:
                json.dump(self.metrics, f, indent=2)
        
        self.get_logger().info(f'Exported data to {export_path}')
    
    def create_combined_view(self):
        if self.rgb_image is None or self.depth_image is None:
            return None
        
        # Get image dimensions
        height, width = self.rgb_image.shape[:2]
        
        # Create a blank canvas
        combined = np.zeros((height, width * 2, 3), dtype=np.uint8)
        
        # Add RGB image
        combined[:, :width] = self.rgb_image
        
        # Normalize depth image for visualization
        depth_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        # Add depth image
        combined[:, width:] = depth_colormap
        
        # Add labels
        cv2.putText(combined, 'RGB', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined, 'Depth', (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add status information
        status_text = f'Status: {getattr(self, "system_status", "unknown")}'
        scenario_text = f'Scenario: {getattr(self, "current_scenario", "unknown")}'
        cv2.putText(combined, status_text, (10, height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(combined, scenario_text, (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return combined
    
    def create_metrics_visualization(self):
        if not self.metrics:
            return None
        
        # Create a blank canvas
        width, height = 600, 400
        metrics_view = np.ones((height, width, 3), dtype=np.uint8) * 240  # Light gray
        
        # Add title
        cv2.putText(metrics_view, 'Performance Metrics', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        # Add metrics
        y_pos = 100
        for i, (key, value) in enumerate(self.metrics.items()):
            # Format the metric name and value
            metric_name = key.replace('_', ' ').title()
            
            if isinstance(value, float):
                metric_value = f'{value:.2f}'
            else:
                metric_value = str(value)
            
            # Add units based on metric name
            if 'time' in key:
                metric_value += ' s'
            elif 'throughput' in key:
                metric_value += ' items/min'
            elif 'accuracy' in key:
                metric_value += ' %'
            
            # Draw metric
            cv2.putText(metrics_view, metric_name, (50, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
            cv2.putText(metrics_view, metric_value, (350, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
            
            # Draw a bar for percentage metrics
            if 'accuracy' in key:
                bar_length = int(value * 2)  # Scale to fit
                cv2.rectangle(metrics_view, (50, y_pos + 10), (50 + bar_length, y_pos + 30), (0, 255, 0), -1)
                cv2.rectangle(metrics_view, (50, y_pos + 10), (50 + 200, y_pos + 30), (0, 0, 0), 1)
            
            y_pos += 60
        
        # Add timestamp
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(metrics_view, f'Updated: {timestamp}', (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        return metrics_view

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
