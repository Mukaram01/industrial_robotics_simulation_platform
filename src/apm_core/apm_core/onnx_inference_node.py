#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import ThreadPoolExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
from apm_msgs.msg import Detection2D, Detection2DArray, DetectedObject, DetectedObjectArray

class OnnxInferenceNode(Node):
    def __init__(self):
        super().__init__('onnx_inference_node')
        
        # Declare parameters
        self.declare_parameter('config_file', 'default_object_detection_config.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ONNX Runtime session
        self.initialize_onnx_runtime()
        
        # Load class labels
        self.class_labels = self.load_class_labels()
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriptions
        self.rgb_subscription = self.create_subscription(
            Image,
            self.config['camera']['topic'],
            self.rgb_callback,
            sensor_qos
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            self.config['camera']['depth_topic'],
            self.depth_callback,
            sensor_qos
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.config['camera']['camera_info_topic'],
            self.camera_info_callback,
            sensor_qos
        )
        
        # Create publishers
        self.visualization_publisher = self.create_publisher(
            Image,
            self.config['output']['visualization_topic'],
            10
        )
        
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            self.config['output']['detections_topic'],
            10
        )
        
        self.objects_publisher = self.create_publisher(
            DetectedObjectArray,
            self.config['output']['detected_objects_topic'],
            10
        )
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/apm/detection/markers',
            10
        )

        # Initialize state variables
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.camera_info = None
        self.detection_counter = 0

        # Thread pool executor for inference processing
        self.processing_executor = ThreadPoolExecutor(max_workers=2)
        
        self.get_logger().info('ONNX Inference Node initialized')
        
    def load_config(self, config_file):
        """Load configuration from YAML file."""
        try:
            config_path = os.path.join(
                get_package_share_directory('apm_core'),
                'config',
                config_file
            )
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config['object_detection']
        except Exception as e:
            self.get_logger().error(f'Error loading config: {str(e)}')
            # Use default configuration as fallback
            return {
                'model': {
                    'path': '',
                    'input_name': 'input',
                    'output_names': ['detection_boxes', 'detection_scores', 'detection_classes'],
                    'input_size': [300, 300],
                    'confidence_threshold': 0.5
                },
                'classes': {
                    'path': ''
                },
                'processing': {
                    'enable_gpu': False,
                    'execution_provider': 'CPUExecutionProvider',
                    'max_detections': 10
                },
                'camera': {
                    'topic': '/camera/color/image_raw',
                    'depth_topic': '/camera/depth/image_rect_raw',
                    'camera_info_topic': '/camera/color/camera_info'
                },
                'output': {
                    'publish_visualization': True,
                    'visualization_topic': '/apm/detection/visualization',
                    'detections_topic': '/apm/detection/detections',
                    'detected_objects_topic': '/apm/detection/objects'
                }
            }
    
    def initialize_onnx_runtime(self):
        """Initialize ONNX Runtime session with the model."""
        try:
            # Resolve model path
            model_path = self.config['model']['path']
            if '$(find' in model_path:
                # Extract package name
                start_idx = model_path.find('$(find') + 7
                end_idx = model_path.find(')', start_idx)
                package_name = model_path[start_idx:end_idx]
                
                # Get package path
                package_path = get_package_share_directory(package_name)
                
                # Replace placeholder with actual path
                model_path = model_path.replace(
                    f'$(find {package_name})',
                    package_path
                )
            
            # Set execution providers
            providers = []
            if self.config['processing']['enable_gpu']:
                providers.append('CUDAExecutionProvider')
            providers.append(self.config['processing']['execution_provider'])
            
            # Create session options
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            
            # Create inference session
            self.session = ort.InferenceSession(
                model_path,
                sess_options=sess_options,
                providers=providers
            )
            
            self.get_logger().info(f'ONNX model loaded from: {model_path}')
            self.get_logger().info(f'Using providers: {providers}')
            
        except Exception as e:
            self.get_logger().error(f'Error initializing ONNX Runtime: {str(e)}')
            self.session = None
    
    def load_class_labels(self):
        """Load class labels from file."""
        try:
            # Resolve class labels path
            labels_path = self.config['classes']['path']
            if '$(find' in labels_path:
                # Extract package name
                start_idx = labels_path.find('$(find') + 7
                end_idx = labels_path.find(')', start_idx)
                package_name = labels_path[start_idx:end_idx]
                
                # Get package path
                package_path = get_package_share_directory(package_name)
                
                # Replace placeholder with actual path
                labels_path = labels_path.replace(
                    f'$(find {package_name})',
                    package_path
                )
            
            # Load labels
            with open(labels_path, 'r') as f:
                labels = [line.strip() for line in f.readlines()]
            
            self.get_logger().info(f'Loaded {len(labels)} class labels from: {labels_path}')
            return labels
            
        except Exception as e:
            self.get_logger().error(f'Error loading class labels: {str(e)}')
            # Return some default COCO classes as fallback
            return [
                'background', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
                'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
                'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
                'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
                'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
                'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
                'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
            ]
    
    def rgb_callback(self, msg):
        """Schedule inference on a background thread."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_image = cv_image

            if self.session is not None and self.camera_info is not None:
                self.processing_executor.submit(self.perform_inference, cv_image, msg.header)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
    
    def depth_callback(self, msg):
        """Store depth image for 3D position estimation."""
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def camera_info_callback(self, msg):
        """Store camera info for 3D position estimation."""
        self.camera_info = msg
    
    def perform_inference(self, image, header):
        """Perform object detection inference on the image."""
        if self.session is None:
            self.get_logger().warn('ONNX session not initialized, skipping inference')
            return
        
        try:
            # Preprocess image
            input_size = self.config['model']['input_size']
            preprocessed_image = self.preprocess_image(image, input_size)
            
            # Run inference
            input_name = self.config['model']['input_name']
            output_names = self.config['model']['output_names']
            
            outputs = self.session.run(
                output_names,
                {input_name: preprocessed_image}
            )
            
            # Process detections
            detections = self.process_detections(outputs, image.shape)
            
            # Create visualization
            if self.config['output']['publish_visualization']:
                visualization = self.create_visualization(image, detections)
                self.publish_visualization(visualization, header)
            
            # Publish 2D detections
            self.publish_detections_2d(detections, header)
            
            # Estimate 3D positions and publish objects
            if self.latest_depth_image is not None and self.camera_info is not None:
                objects = self.estimate_3d_positions(detections, header)
                self.publish_detected_objects(objects, header)
                self.publish_markers(objects, header)
            
        except Exception as e:
            self.get_logger().error(f'Error during inference: {str(e)}')
    
    def preprocess_image(self, image, input_size):
        """Preprocess image for the ONNX model."""
        # Resize image
        resized = cv2.resize(image, (input_size[0], input_size[1]))
        
        # Convert to RGB (if model expects RGB)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize pixel values to [0, 1]
        normalized = rgb.astype(np.float32) / 255.0
        
        # Add batch dimension
        batched = np.expand_dims(normalized, axis=0)
        
        return batched
    
    def process_detections(self, outputs, image_shape):
        """Process model outputs to get detections."""
        # Extract outputs
        boxes = outputs[0][0]  # Assuming first output is boxes
        scores = outputs[1][0]  # Assuming second output is scores
        classes = outputs[2][0]  # Assuming third output is classes
        
        # Filter by confidence threshold
        threshold = self.config['model']['confidence_threshold']
        max_detections = self.config['processing']['max_detections']
        
        detections = []
        for i in range(min(len(scores), max_detections)):
            if scores[i] >= threshold:
                # Get class info
                class_id = int(classes[i])
                if class_id < len(self.class_labels):
                    class_name = self.class_labels[class_id]
                else:
                    class_name = f"class_{class_id}"
                
                # Get normalized box coordinates [y1, x1, y2, x2]
                box = boxes[i]
                
                # Convert to pixel coordinates [x1, y1, x2, y2]
                height, width = image_shape[:2]
                x1 = int(box[1] * width)
                y1 = int(box[0] * height)
                x2 = int(box[3] * width)
                y2 = int(box[2] * height)
                
                # Create detection
                detection = {
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': float(scores[i]),
                    'box': [x1, y1, x2, y2]
                }
                
                detections.append(detection)
        
        return detections
    
    def create_visualization(self, image, detections):
        """Create visualization image with detection boxes."""
        # Create a copy of the image
        vis_image = image.copy()
        
        # Draw each detection
        for detection in detections:
            # Get box coordinates
            x1, y1, x2, y2 = detection['box']
            
            # Get class info
            class_name = detection['class_name']
            confidence = detection['confidence']
            
            # Generate color based on class ID
            color = self.get_color_for_class(detection['class_id'])
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label background
            label = f"{class_name}: {confidence:.2f}"
            size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.rectangle(vis_image, (x1, y1 - size[1] - 5), (x1 + size[0], y1), color, -1)
            
            # Draw label text
            cv2.putText(vis_image, label, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return vis_image
    
    def get_color_for_class(self, class_id):
        """Generate a consistent color for a class ID."""
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Cyan
            (255, 128, 0),  # Orange
            (128, 0, 255),  # Purple
            (0, 128, 255),  # Light Blue
            (255, 0, 128)   # Pink
        ]
        return colors[class_id % len(colors)]
    
    def publish_visualization(self, image, header):
        """Publish visualization image."""
        try:
            # Convert OpenCV image to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            img_msg.header = header
            
            # Publish message
            self.visualization_publisher.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing visualization: {str(e)}')
    
    def publish_detections_2d(self, detections, header):
        """Publish 2D detection results."""
        try:
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = header
            
            # Add each detection
            for detection in detections:
                # Create Detection2D message
                det_msg = Detection2D()
                det_msg.header = header
                
                # Set box coordinates
                x1, y1, x2, y2 = detection['box']
                det_msg.bbox.x = x1
                det_msg.bbox.y = y1
                det_msg.bbox.width = x2 - x1
                det_msg.bbox.height = y2 - y1
                
                # Set class info
                det_msg.class_id = detection['class_id']
                det_msg.class_name = detection['class_name']
                det_msg.score = detection['confidence']
                
                # Add to array
                detection_array.detections.append(det_msg)
            
            # Publish message
            self.detections_publisher.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing 2D detections: {str(e)}')
    
    def estimate_3d_positions(self, detections, header):
        """Estimate 3D positions of detected objects using depth image."""
        objects = []
        
        if self.latest_depth_image is None or self.camera_info is None:
            return objects
        
        try:
            # Get camera intrinsics
            fx = self.camera_info.k[0]  # Focal length x
            fy = self.camera_info.k[4]  # Focal length y
            cx = self.camera_info.k[2]  # Principal point x
            cy = self.camera_info.k[5]  # Principal point y
            
            # Process each detection
            for detection in detections:
                # Get box coordinates
                x1, y1, x2, y2 = detection['box']
                
                # Calculate center point of the box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                # Get depth at center point (in meters)
                # Note: This assumes depth is in millimeters and needs conversion
                if (0 <= center_y < self.latest_depth_image.shape[0] and 
                    0 <= center_x < self.latest_depth_image.shape[1]):
                    
                    depth_value = self.latest_depth_image[center_y, center_x]
                    
                    # Skip invalid depth values
                    if depth_value == 0 or np.isnan(depth_value):
                        continue
                    
                    # Convert depth to meters (assuming depth is in millimeters)
                    depth_meters = depth_value / 1000.0
                    
                    # Calculate 3D coordinates
                    x = (center_x - cx) * depth_meters / fx
                    y = (center_y - cy) * depth_meters / fy
                    z = depth_meters
                    
                    # Create object
                    obj = {
                        'class_id': detection['class_id'],
                        'class_name': detection['class_name'],
                        'confidence': detection['confidence'],
                        'box': detection['box'],
                        'position': [x, y, z],
                        'id': self.detection_counter
                    }
                    
                    objects.append(obj)
                    self.detection_counter += 1
            
        except Exception as e:
            self.get_logger().error(f'Error estimating 3D positions: {str(e)}')
        
        return objects
    
    def publish_detected_objects(self, objects, header):
        """Publish 3D detected objects."""
        try:
            # Create DetectedObjectArray message
            object_array = DetectedObjectArray()
            object_array.header = header
            
            # Add each object
            for obj in objects:
                # Create DetectedObject message
                obj_msg = DetectedObject()
                obj_msg.header = header
                
                # Set object ID
                obj_msg.id = obj['id']
                
                # Set class info
                obj_msg.class_id = obj['class_id']
                obj_msg.class_name = obj['class_name']
                obj_msg.confidence = obj['confidence']
                
                # Set 3D position
                obj_msg.pose.position.x = obj['position'][0]
                obj_msg.pose.position.y = obj['position'][1]
                obj_msg.pose.position.z = obj['position'][2]
                
                # Set orientation (identity quaternion)
                obj_msg.pose.orientation.w = 1.0
                
                # Set dimensions (estimated from bounding box)
                x1, y1, x2, y2 = obj['box']
                width_pixels = x2 - x1
                height_pixels = y2 - y1
                
                # Rough estimate of physical size based on depth
                depth = obj['position'][2]
                obj_msg.dimensions.x = width_pixels * depth / self.camera_info.k[0]  # width
                obj_msg.dimensions.y = height_pixels * depth / self.camera_info.k[4]  # height
                obj_msg.dimensions.z = min(obj_msg.dimensions.x, obj_msg.dimensions.y)  # depth (estimate)
                
                # Add to array
                object_array.objects.append(obj_msg)
            
            # Publish message
            self.objects_publisher.publish(object_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing 3D objects: {str(e)}')
    
    def publish_markers(self, objects, header):
        """Publish visualization markers for detected objects."""
        try:
            # Create MarkerArray message
            marker_array = MarkerArray()
            
            # Add each object
            for obj in objects:
                # Create cube marker
                cube_marker = Marker()
                cube_marker.header = header
                cube_marker.ns = "objects"
                cube_marker.id = obj['id']
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                
                # Set position
                cube_marker.pose.position.x = obj['position'][0]
                cube_marker.pose.position.y = obj['position'][1]
                cube_marker.pose.position.z = obj['position'][2]
                
                # Set orientation (identity quaternion)
                cube_marker.pose.orientation.w = 1.0
                
                # Set dimensions (estimated from bounding box)
                x1, y1, x2, y2 = obj['box']
                width_pixels = x2 - x1
                height_pixels = y2 - y1
                
                # Rough estimate of physical size based on depth
                depth = obj['position'][2]
                cube_marker.scale.x = width_pixels * depth / self.camera_info.k[0]  # width
                cube_marker.scale.y = height_pixels * depth / self.camera_info.k[4]  # height
                cube_marker.scale.z = min(cube_marker.scale.x, cube_marker.scale.y)  # depth (estimate)
                
                # Set color based on class
                color = self.get_color_for_class(obj['class_id'])
                cube_marker.color.r = color[0] / 255.0
                cube_marker.color.g = color[1] / 255.0
                cube_marker.color.b = color[2] / 255.0
                cube_marker.color.a = 0.7  # Semi-transparent
                
                # Set lifetime
                cube_marker.lifetime.sec = 1
                
                # Add to array
                marker_array.markers.append(cube_marker)
                
                # Create text marker for label
                text_marker = Marker()
                text_marker.header = header
                text_marker.ns = "labels"
                text_marker.id = obj['id']
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Set position (slightly above the object)
                text_marker.pose.position.x = obj['position'][0]
                text_marker.pose.position.y = obj['position'][1]
                text_marker.pose.position.z = obj['position'][2] + cube_marker.scale.z/2 + 0.05
                
                # Set orientation (identity quaternion)
                text_marker.pose.orientation.w = 1.0
                
                # Set scale (text height)
                text_marker.scale.z = 0.05
                
                # Set color (white)
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                # Set text
                text_marker.text = f"{obj['class_name']}: {obj['confidence']:.2f}"
                
                # Set lifetime
                text_marker.lifetime.sec = 1
                
                # Add to array
                marker_array.markers.append(text_marker)
            
            # Publish message
            self.marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing markers: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OnnxInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.processing_executor.shutdown()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
