#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from apm_msgs.msg import Detection2D, Detection2DArray

class SegmentationNode(Node):
    """
    Node for image segmentation to identify object boundaries
    """
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('segmentation_config', '')
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.segmentation_config_path = self.get_parameter('segmentation_config').value
        
        # Load configuration
        self.load_config()
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Create publishers
        self.segmented_image_pub = self.create_publisher(
            Image,
            '/apm/advanced_perception/segmented_image',
            10
        )
        
        self.segmented_objects_pub = self.create_publisher(
            Detection2DArray,
            '/apm/advanced_perception/segmented_objects',
            10
        )
        
        self.get_logger().info('Segmentation node initialized')
    
    def load_config(self):
        """
        Load segmentation configuration from YAML file
        """
        if not self.segmentation_config_path:
            self.get_logger().warn('No segmentation config file provided, using default values')
            self.threshold_min = np.array([0, 0, 0])
            self.threshold_max = np.array([255, 255, 255])
            self.min_contour_area = 1000
            return
        
        try:
            with open(self.segmentation_config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.threshold_min = np.array(config.get('threshold_min', [0, 0, 0]))
            self.threshold_max = np.array(config.get('threshold_max', [255, 255, 255]))
            self.min_contour_area = config.get('min_contour_area', 1000)
            
            self.get_logger().info(f'Loaded segmentation config from {self.segmentation_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load segmentation config: {str(e)}')
            self.threshold_min = np.array([0, 0, 0])
            self.threshold_max = np.array([255, 255, 255])
            self.min_contour_area = 1000
    
    def image_callback(self, msg):
        """
        Process incoming image and perform segmentation
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Perform segmentation
            segmented_image, objects = self.segment_image(cv_image)
            
            # Publish segmented image
            segmented_image_msg = self.cv_bridge.cv2_to_imgmsg(segmented_image, 'bgr8')
            segmented_image_msg.header = msg.header
            self.segmented_image_pub.publish(segmented_image_msg)
            
            # Publish segmented objects
            objects_msg = Detection2DArray()
            objects_msg.header = msg.header
            objects_msg.detections = objects
            self.segmented_objects_pub.publish(objects_msg)
            
            self.get_logger().debug(f'Processed image, found {len(objects)} objects')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def segment_image(self, image):
        """
        Segment image to identify objects
        
        Args:
            image: OpenCV image in BGR format
            
        Returns:
            segmented_image: Image with segmentation visualization
            objects: List of Detection2D messages
        """
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask using threshold values
        mask = cv2.inRange(hsv, self.threshold_min, self.threshold_max)
        
        # Apply morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create output image for visualization
        segmented_image = image.copy()
        
        # Create list for detected objects
        objects = []
        
        # Process each contour
        for i, contour in enumerate(contours):
            # Filter small contours
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue
            
            # Draw contour on visualization image
            cv2.drawContours(segmented_image, [contour], 0, (0, 255, 0), 2)
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw bounding box
            cv2.rectangle(segmented_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            # Calculate center
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Draw center
            cv2.circle(segmented_image, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Create detection message
            obj = Detection2D()
            obj.class_id = 0
            obj.class_name = 'segmented_object'
            obj.score = float(area) / (image.shape[0] * image.shape[1])

            # Set bounding box
            obj.bbox.x_offset = int(x)
            obj.bbox.y_offset = int(y)
            obj.bbox.width = int(w)
            obj.bbox.height = int(h)
            obj.bbox.do_rectify = False
            
            # Add to list
            objects.append(obj)
        
        return segmented_image, objects

def main(args=None):
    rclpy.init(args=args)
    
    segmentation_node = SegmentationNode()
    
    try:
        rclpy.spin(segmentation_node)
    except KeyboardInterrupt:
        pass
    finally:
        segmentation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
