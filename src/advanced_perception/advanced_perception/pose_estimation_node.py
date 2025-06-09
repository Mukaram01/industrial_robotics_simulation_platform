#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import ThreadPoolExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from geometry_msgs.msg import PoseStamped
from apm_msgs.msg import (
    DetectedObject,
    DetectedObjectArray,
    Detection2D,
    Detection2DArray,
)

class PoseEstimationNode(Node):
    """
    Node for 6D pose estimation of detected objects
    """
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pose_estimation_config', '')
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pose_estimation_config_path = self.get_parameter('pose_estimation_config').value
        
        # Load configuration
        self.load_config()
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Initialize camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        self.segmented_objects_sub = self.create_subscription(
            Detection2DArray,
            '/apm/advanced_perception/segmented_objects',
            self.segmented_objects_callback,
            10
        )
        
        # Create publishers
        self.objects_pub = self.create_publisher(
            DetectedObjectArray,
            '/apm/advanced_perception/objects',
            10
        )

        # Initialize state
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_segmented_objects = None

        # Thread pool executor for processing
        self.processing_executor = ThreadPoolExecutor(max_workers=2)

        self.get_logger().info('Pose estimation node initialized')
    
    def load_config(self):
        """
        Load pose estimation configuration from YAML file
        """
        if not self.pose_estimation_config_path:
            self.get_logger().warn('No pose estimation config file provided, using default values')
            self.min_depth = 0.1
            self.max_depth = 5.0
            self.depth_scale = 0.001  # Convert mm to meters
            return

        if not os.path.isfile(self.pose_estimation_config_path):
            self.get_logger().error(f'Pose estimation config file not found: {self.pose_estimation_config_path}')
            self.min_depth = 0.1
            self.max_depth = 5.0
            self.depth_scale = 0.001
            return
        
        try:
            with open(self.pose_estimation_config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.min_depth = config.get('min_depth', 0.1)
            self.max_depth = config.get('max_depth', 5.0)
            self.depth_scale = config.get('depth_scale', 0.001)
            
            self.get_logger().info(f'Loaded pose estimation config from {self.pose_estimation_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load pose estimation config: {str(e)}')
            self.min_depth = 0.1
            self.max_depth = 5.0
            self.depth_scale = 0.001
    
    def camera_info_callback(self, msg):
        """
        Process camera info message to extract intrinsics
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().debug('Received camera intrinsics')
    
    def image_callback(self, msg):
        """
        Process incoming RGB image
        """
        try:
            self.latest_rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.processing_executor.submit(self.process_data)
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
    
    def depth_callback(self, msg):
        """
        Process incoming depth image
        """
        try:
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.processing_executor.submit(self.process_data)
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def segmented_objects_callback(self, msg):
        """
        Process segmented objects message
        """
        self.latest_segmented_objects = msg
        self.processing_executor.submit(self.process_data)
    
    def process_data(self):
        """
        Process all available data to estimate object poses
        """
        # Check if all required data is available
        if (self.latest_rgb_image is None or 
            self.latest_depth_image is None or 
            self.latest_segmented_objects is None or
            self.camera_matrix is None):
            return
        
        try:
            # Create output message
            objects_msg = DetectedObjectArray()
            objects_msg.header = self.latest_segmented_objects.header

            # Process each segmented object
            for idx, segmented_obj in enumerate(self.latest_segmented_objects.detections):
                # Get object region from RGB and depth images
                x = int(segmented_obj.bbox.x_offset)
                y = int(segmented_obj.bbox.y_offset)
                w = int(segmented_obj.bbox.width)
                h = int(segmented_obj.bbox.height)
                
                # Ensure bounds are within image
                x = max(0, x)
                y = max(0, y)
                w = min(w, self.latest_rgb_image.shape[1] - x)
                h = min(h, self.latest_rgb_image.shape[0] - y)
                
                # Skip if region is too small
                if w <= 0 or h <= 0:
                    continue
                
                # Get depth in object region
                depth_roi = self.latest_depth_image[y:y+h, x:x+w]
                
                # Filter invalid depths
                valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < 65535)]
                
                if len(valid_depths) == 0:
                    continue
                
                # Calculate median depth (more robust than mean)
                median_depth = np.median(valid_depths) * self.depth_scale
                
                # Skip if depth is out of range
                if median_depth < self.min_depth or median_depth > self.max_depth:
                    continue
                
                # Calculate 3D position
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Convert from image coordinates to camera coordinates
                if self.camera_matrix is not None:
                    # Get focal length and principal point
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    cx = self.camera_matrix[0, 2]
                    cy = self.camera_matrix[1, 2]
                    
                    # Calculate 3D position in camera frame
                    X = (center_x - cx) * median_depth / fx
                    Y = (center_y - cy) * median_depth / fy
                    Z = median_depth
                else:
                    # Fallback if camera matrix is not available
                    X = 0.0
                    Y = 0.0
                    Z = median_depth
                
                # Create detected object with pose
                obj = DetectedObject()
                obj.header = self.latest_segmented_objects.header
                obj.id = idx + 1
                obj.class_id = segmented_obj.class_id
                obj.class_name = segmented_obj.class_name
                obj.confidence = segmented_obj.score

                # Set 3D position
                obj.pose.position.x = X
                obj.pose.position.y = Y
                obj.pose.position.z = Z

                # Set orientation (identity quaternion for now)
                obj.pose.orientation.w = 1.0
                obj.pose.orientation.x = 0.0
                obj.pose.orientation.y = 0.0
                obj.pose.orientation.z = 0.0

                # Set dimensions based on depth and bounding box
                obj.dimensions.x = w * median_depth / fx
                obj.dimensions.y = h * median_depth / fy
                obj.dimensions.z = 0.1  # Default depth
                
                # Add to output message
                objects_msg.objects.append(obj)
            
            # Publish objects with poses
            self.objects_pub.publish(objects_msg)
            self.get_logger().debug(f'Published {len(objects_msg.objects)} objects with poses')
            
        except Exception as e:
            self.get_logger().error(f'Error estimating poses: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    pose_estimation_node = PoseEstimationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(pose_estimation_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pose_estimation_node.processing_executor.shutdown()
        pose_estimation_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
