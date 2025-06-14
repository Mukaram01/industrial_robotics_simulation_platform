#!/usr/bin/env python3
"""Standalone script for testing the image subscriber node."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # Create subscription
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Image subscriber initialized. Listening on topic: {image_topic}')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Log image information
            self.get_logger().info(f'Received image: {cv_image.shape}')
            
            # Display image (optional - for debugging)
            # cv2.imshow("Camera Image", cv_image)
            # cv2.waitKey(1)
            
            # Process image here (add your perception code)
            # Example: simple color detection
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # Add more processing as needed
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    
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
