#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from apm_msgs.msg import DetectedObjectArray
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class PlanningSceneUpdaterNode(Node):
    """
    Node for updating the MoveIt2 planning scene with detected objects
    """
    def __init__(self):
        super().__init__('planning_scene_updater_node')
        
        # Declare parameters
        self.declare_parameter('update_frequency', 10.0)  # Hz
        self.declare_parameter('detected_objects_topic', '/apm/detection/objects')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('object_padding', 0.02)  # meters
        
        # Get parameters
        self.update_frequency = self.get_parameter('update_frequency').value
        self.detected_objects_topic = self.get_parameter('detected_objects_topic').value
        self.planning_scene_topic = self.get_parameter('planning_scene_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.object_padding = self.get_parameter('object_padding').value
        
        # Create callback group for services and subscribers
        self.callback_group = ReentrantCallbackGroup()
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create planning scene publisher
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            self.planning_scene_topic,
            10
        )
        
        # Create detected objects subscriber
        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray,
            self.detected_objects_topic,
            self.detected_objects_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create planning scene service client
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene,
            '/get_planning_scene',
            callback_group=self.callback_group
        )
        
        # Initialize object tracking
        self.detected_object_ids = set()
        self.object_lock = threading.Lock()
        
        # Create timer for periodic updates
        self.update_timer = self.create_timer(
            1.0 / self.update_frequency,
            self.update_planning_scene,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Planning Scene Updater Node initialized')
    
    def detected_objects_callback(self, msg):
        """
        Callback for processing detected objects
        """
        with self.object_lock:
            # Clear previous objects
            self.detected_object_ids = set()
            
            # Process new detections
            for obj in msg.objects:
                # Add to tracking
                self.detected_object_ids.add(obj.id)
                
                try:
                    # Transform pose to planning frame if needed
                    if obj.pose.header.frame_id != self.world_frame:
                        transform = self.tf_buffer.lookup_transform(
                            self.world_frame,
                            obj.pose.header.frame_id,
                            rclpy.time.Time()
                        )
                        transformed_pose = do_transform_pose(obj.pose.pose, transform)
                    else:
                        transformed_pose = obj.pose.pose
                    
                    # Create collision object
                    co = CollisionObject()
                    co.header.frame_id = self.world_frame
                    co.header.stamp = self.get_clock().now().to_msg()
                    co.id = f"object_{obj.id}"
                    co.operation = CollisionObject.ADD
                    
                    # Create primitive based on object type
                    primitive = SolidPrimitive()
                    if obj.shape == "box":
                        primitive.type = SolidPrimitive.BOX
                        primitive.dimensions = [
                            obj.dimensions.x + self.object_padding,
                            obj.dimensions.y + self.object_padding,
                            obj.dimensions.z + self.object_padding
                        ]
                    elif obj.shape == "cylinder":
                        primitive.type = SolidPrimitive.CYLINDER
                        primitive.dimensions = [
                            obj.dimensions.z + self.object_padding,  # height
                            obj.dimensions.x / 2.0 + self.object_padding  # radius
                        ]
                    elif obj.shape == "sphere":
                        primitive.type = SolidPrimitive.SPHERE
                        primitive.dimensions = [
                            obj.dimensions.x / 2.0 + self.object_padding  # radius
                        ]
                    else:
                        # Default to box
                        primitive.type = SolidPrimitive.BOX
                        primitive.dimensions = [
                            obj.dimensions.x + self.object_padding,
                            obj.dimensions.y + self.object_padding,
                            obj.dimensions.z + self.object_padding
                        ]
                    
                    co.primitives.append(primitive)
                    co.primitive_poses.append(transformed_pose)
                    
                    # Add to planning scene
                    ps = PlanningScene()
                    ps.is_diff = True
                    ps.world.collision_objects.append(co)
                    self.planning_scene_pub.publish(ps)
                    
                except Exception as e:
                    self.get_logger().error(f"Error adding object to planning scene: {str(e)}")
    
    def update_planning_scene(self):
        """
        Periodic update of the planning scene
        """
        try:
            # Request current planning scene with world objects
            req = GetPlanningScene.Request()
            req.components = PlanningSceneComponents()
            req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES |
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )

            future = self.get_planning_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            response = future.result()
            if response is None:
                self.get_logger().warning('Failed to retrieve planning scene')
                return

            current_ids = {
                co.id for co in response.scene.world.collision_objects
            }

            with self.object_lock:
                tracked_ids = {
                    f"object_{oid}" for oid in self.detected_object_ids
                }

            ids_to_remove = current_ids - tracked_ids

            for obj_id in ids_to_remove:
                co = CollisionObject()
                co.header.frame_id = self.world_frame
                co.id = obj_id
                co.operation = CollisionObject.REMOVE

                ps = PlanningScene()
                ps.is_diff = True
                ps.world.collision_objects.append(co)
                self.planning_scene_pub.publish(ps)

        except Exception as e:
            self.get_logger().error(
                f'Error updating planning scene: {str(e)}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    planning_scene_updater = PlanningSceneUpdaterNode()
    
    # Use multithreaded executor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(planning_scene_updater)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        planning_scene_updater.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
