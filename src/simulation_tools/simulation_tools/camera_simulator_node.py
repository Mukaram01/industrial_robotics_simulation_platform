#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSimulatorNode(Node):
    def __init__(self):
        super().__init__('camera_simulator_node')

        # Declare parameters using a single dictionary
        param_defaults = {
            'simulation_mode': 'synthetic',
            'frame_rate': 30.0,
            'resolution_width': 640,
            'resolution_height': 480,
            'camera_name': 'camera',
            'object_count': 5,
            'object_types': ['red_cube', 'green_cylinder', 'blue_sphere'],
            'background_type': 'conveyor_belt',
            'noise_level': 0.02,
            'simulate_lighting': True,
            'simulate_occlusion': False,
            'publish_compressed': False,
            'compression_format': 'jpeg',
        }
        self.declare_parameters('', [(k, v) for k, v in param_defaults.items()])

        # Get parameters
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        self.camera_name = self.get_parameter('camera_name').value
        self.object_count = self.get_parameter('object_count').value
        self.object_types = self.get_parameter('object_types').value
        self.background_type = self.get_parameter('background_type').value
        self.noise_level = self.get_parameter('noise_level').value
        self.simulate_lighting = self.get_parameter('simulate_lighting').value
        self.simulate_occlusion = self.get_parameter('simulate_occlusion').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.compression_format = self.get_parameter('compression_format').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create publishers
        self.rgb_pub = self.create_publisher(
            Image,
            f'/{self.camera_name}/color/image_raw',
            10)
        self.depth_pub = self.create_publisher(
            Image,
            f'/{self.camera_name}/depth/image_rect_raw',
            10)
        if self.publish_compressed:
            self.rgb_compressed_pub = self.create_publisher(
                CompressedImage,
                f'/{self.camera_name}/color/image_raw/compressed',
                10)
            self.depth_compressed_pub = self.create_publisher(
                CompressedImage,
                f'/{self.camera_name}/depth/image_rect_raw/compressed',
                10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            f'/{self.camera_name}/color/camera_info',
            10)

        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            '/simulation/command',
            self.command_callback,
            10)

        # Initialize camera info
        self.camera_info = self.create_camera_info()

        # Initialize simulation state
        self.running = False
        self.objects = []
        self.generate_objects()

        # Start publishing timer
        self.timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Camera simulator node initialized')

    def command_callback(self, msg):
        command = msg.data
        if command == 'start':
            self.running = True
            self.get_logger().info('Starting camera simulation')
        elif command == 'stop':
            self.running = False
            self.get_logger().info('Stopping camera simulation')
        elif command == 'reset':
            self.generate_objects()
            self.get_logger().info('Resetting camera simulation')

    def timer_callback(self):
        if not self.running:
            return

        # Generate images
        rgb_image, depth_image = self.generate_images()

        # Add noise if specified
        if self.noise_level > 0:
            rgb_image = self.add_noise(rgb_image, self.noise_level)
            depth_image = self.add_noise(depth_image, self.noise_level)

        # Convert to ROS messages
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')

        # Set headers
        now = self.get_clock().now().to_msg()
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = f'{self.camera_name}_color_optical_frame'
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = f'{self.camera_name}_depth_optical_frame'

        # Update camera info timestamp
        self.camera_info.header.stamp = now

        # Publish messages
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(self.camera_info)

        if self.publish_compressed:
            rgb_comp_msg = CompressedImage()
            rgb_comp_msg.header = rgb_msg.header
            rgb_comp_msg.format = self.compression_format
            success, buffer = cv2.imencode(
                '.jpg' if self.compression_format == 'jpeg' else '.png',
                rgb_image,
            )
            if success:
                rgb_comp_msg.data = buffer.tobytes()
                self.rgb_compressed_pub.publish(rgb_comp_msg)

            depth_comp_msg = CompressedImage()
            depth_comp_msg.header = depth_msg.header
            depth_comp_msg.format = 'png'
            depth_uint16 = np.clip(depth_image * 1000.0, 0, 65535).astype(np.uint16)
            success, buffer = cv2.imencode('.png', depth_uint16)
            if success:
                depth_comp_msg.data = buffer.tobytes()
                self.depth_compressed_pub.publish(depth_comp_msg)

    def generate_images(self):
        # Create background
        rgb_image = self.create_background(self.width, self.height, self.background_type)
        depth_image = np.ones((self.height, self.width), dtype=np.float32) * 10.0  # Initialize with far distance

        # Update object positions
        self.update_objects()

        # Render objects
        for obj in self.objects:
            self.render_object(obj, rgb_image, depth_image)

        return rgb_image, depth_image

    def create_background(self, width, height, background_type):
        if background_type == 'conveyor_belt':
            # Create a gray conveyor belt with some texture
            image = np.ones((height, width, 3), dtype=np.uint8) * 100  # Gray background

            # Add conveyor belt texture
            for i in range(0, width, 20):
                cv2.line(image, (i, 0), (i, height), (80, 80, 80), 1)

            # Add conveyor belt edges
            cv2.rectangle(image, (0, 0), (width-1, height-1), (50, 50, 50), 10)

            return image
        elif background_type == 'table':
            # Create a wooden table texture
            image = np.ones((height, width, 3), dtype=np.uint8) * 180  # Light brown

            # Add wood grain
            for i in range(0, height, 15):
                cv2.line(image, (0, i), (width, i), (160, 120, 80), 2)

            return image
        else:
            # Default plain background
            return np.ones((height, width, 3), dtype=np.uint8) * 240  # Light gray

    def generate_objects(self):
        self.objects = []
        for i in range(self.object_count):
            obj_type = np.random.choice(self.object_types)

            # Parse type and color from string (e.g., "red_cube")
            parts = obj_type.split('_')
            color_name = parts[0]
            shape = parts[1] if len(parts) > 1 else 'cube'

            # Map color name to BGR values
            color_map = {
                'red': (0, 0, 255),
                'green': (0, 255, 0),
                'blue': (255, 0, 0),
                'yellow': (0, 255, 255),
                'cyan': (255, 255, 0),
                'magenta': (255, 0, 255),
                'white': (255, 255, 255),
                'black': (0, 0, 0)
            }
            color = color_map.get(color_name, (128, 128, 128))

            # Create object
            obj = {
                'id': f'obj_{i}',
                'type': shape,
                'color': color,
                'position': [
                    np.random.uniform(0.2, 0.8) * self.width,
                    np.random.uniform(0.2, 0.8) * self.height,
                    np.random.uniform(0.5, 1.5)  # depth in meters
                ],
                'size': np.random.uniform(30, 60),
                'velocity': [
                    np.random.uniform(-2, 2),
                    np.random.uniform(-2, 2),
                    0
                ]
            }
            self.objects.append(obj)

    def update_objects(self):
        for obj in self.objects:
            # Update position based on velocity
            obj['position'][0] += obj['velocity'][0]
            obj['position'][1] += obj['velocity'][1]

            # Bounce off edges
            if obj['position'][0] < 50 or obj['position'][0] > self.width - 50:
                obj['velocity'][0] *= -1
            if obj['position'][1] < 50 or obj['position'][1] > self.height - 50:
                obj['velocity'][1] *= -1

    def render_object(self, obj, rgb_image, depth_image):
        x, y, z = int(obj['position'][0]), int(obj['position'][1]), obj['position'][2]
        size = int(obj['size'])
        color = obj['color']

        if obj['type'] == 'cube':
            # Draw a rectangle
            half_size = size // 2
            cv2.rectangle(rgb_image,
                         (x - half_size, y - half_size),
                         (x + half_size, y + half_size),
                         color, -1)

            # Update depth
            depth_image[y-half_size:y+half_size, x-half_size:x+half_size] = z

        elif obj['type'] == 'cylinder':
            # Draw a circle (top view of cylinder)
            cv2.circle(rgb_image, (x, y), size // 2, color, -1)

            # Update depth
            cv2.circle(depth_image, (x, y), size // 2, z, -1)

        elif obj['type'] == 'sphere':
            # Draw a circle with shading to look like a sphere
            cv2.circle(rgb_image, (x, y), size // 2, color, -1)

            # Add highlight for 3D effect if lighting is enabled
            if self.simulate_lighting:
                highlight_size = size // 4
                highlight_pos = (x - highlight_size // 2, y - highlight_size // 2)
                cv2.circle(rgb_image, highlight_pos, highlight_size,
                          (min(color[0] + 50, 255),
                           min(color[1] + 50, 255),
                           min(color[2] + 50, 255)), -1)

            # Create gradient depth for sphere
            for i in range(y - size // 2, y + size // 2):
                for j in range(x - size // 2, x + size // 2):
                    if 0 <= i < self.height and 0 <= j < self.width:
                        dist_sq = (j - x) ** 2 + (i - y) ** 2
                        if dist_sq < (size // 2) ** 2:
                            # Calculate depth based on sphere equation
                            depth_offset = np.sqrt((size // 2) ** 2 - dist_sq) / (size // 2) * 0.1
                            depth_image[i, j] = z - depth_offset

    def add_noise(self, image, noise_level):
        if len(image.shape) == 3:  # RGB image
            noise = np.random.normal(0, noise_level * 255, image.shape).astype(np.int16)
            noisy_image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            return noisy_image
        else:  # Depth image
            noise = np.random.normal(0, noise_level, image.shape)
            noisy_image = np.clip(image + noise, 0, 10.0).astype(np.float32)
            return noisy_image

    def create_camera_info(self):
        camera_info = CameraInfo()
        camera_info.header.frame_id = f'{self.camera_name}_color_optical_frame'

        # Set resolution
        camera_info.width = self.width
        camera_info.height = self.height

        # Set camera matrix (intrinsics)
        fx = self.width * 0.8  # focal length x
        fy = self.width * 0.8  # focal length y
        cx = self.width / 2    # optical center x
        cy = self.height / 2   # optical center y

        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # Set projection matrix
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Set distortion model and parameters (no distortion)
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return camera_info

def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
