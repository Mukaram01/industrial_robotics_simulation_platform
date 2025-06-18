#!/usr/bin/env python3
"""ROS2 node executing pick-and-place operations with MoveIt2."""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header
from apm_msgs.msg import DetectedObject, DetectedObjectArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import moveit_commander
import sys
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetPositionIK

class PickAndPlaceNode(Node):
    """
    Node for executing pick and place operations with the Delta robot
    """
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'delta_robot')
        self.declare_parameter('planning_group', 'delta_arm')
        self.declare_parameter('end_effector_link', 'end_effector')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('detected_objects_topic', '/apm/detection/objects')
        self.declare_parameter('pick_command_topic', '/fmm/pick_command')
        self.declare_parameter('place_command_topic', '/fmm/place_command')
        self.declare_parameter('status_topic', '/fmm/status')
        self.declare_parameter('pre_grasp_distance', 0.1)  # 10cm above object
        self.declare_parameter('post_grasp_distance', 0.1)  # 10cm above grasp
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('max_velocity_scaling_factor', 0.5)
        self.declare_parameter('max_acceleration_scaling_factor', 0.5)
        self.declare_parameter('workspace_limits', [-0.5, 0.5, -0.5, 0.5, 0.0, 0.5])  # [x_min, x_max, y_min, y_max, z_min, z_max]
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_link = self.get_parameter('base_link').value
        self.detected_objects_topic = self.get_parameter('detected_objects_topic').value
        self.pick_command_topic = self.get_parameter('pick_command_topic').value
        self.place_command_topic = self.get_parameter('place_command_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.pre_grasp_distance = self.get_parameter('pre_grasp_distance').value
        self.post_grasp_distance = self.get_parameter('post_grasp_distance').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value
        self.workspace_limits = self.get_parameter('workspace_limits').value
        
        # Create callback groups
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()
        
        # Initialize MoveIt2 Commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)
        # Record the created move group for test access
        setattr(
            moveit_commander.MoveGroupCommander,
            "last_instance",
            self.move_group,
        )
        
        # Set planning parameters
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.num_planning_attempts)
        self.move_group.set_max_velocity_scaling_factor(self.max_velocity_scaling_factor)
        self.move_group.set_max_acceleration_scaling_factor(self.max_acceleration_scaling_factor)
        self.move_group.set_workspace(self.workspace_limits)
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        
        # Create subscribers
        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray,
            self.detected_objects_topic,
            self.detected_objects_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        self.pick_command_sub = self.create_subscription(
            String,
            self.pick_command_topic,
            self.pick_command_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        self.place_command_sub = self.create_subscription(
            String,
            self.place_command_topic,
            self.place_command_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Create service clients
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.service_callback_group
        )
        
        # Initialize object tracking
        self.detected_objects = {}
        self.object_lock = threading.Lock()
        self.current_state = "idle"
        self.attached_object_id = None
        
        # Wait for services
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
        
        self.get_logger().info('Pick and Place Node initialized')
        self.publish_status("ready")
    
    def detected_objects_callback(self, msg):
        """
        Callback for processing detected objects
        """
        with self.object_lock:
            # Update object dictionary
            for obj in msg.objects:
                self.detected_objects[obj.id] = obj
                
                # Add object to planning scene if not already there
                self.add_object_to_planning_scene(obj)
    
    def add_object_to_planning_scene(self, obj):
        """
        Add detected object to the planning scene
        """
        try:
            # Transform object pose to planning frame if needed
            object_pose = obj.pose
            if obj.header.frame_id != self.base_link:
                transform = self.tf_buffer.lookup_transform(
                    self.base_link,
                    obj.header.frame_id,
                    rclpy.time.Time()
                )
                object_pose = do_transform_pose(object_pose, transform)
            
            # Create pose stamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.base_link
            pose_stamped.pose = object_pose
            
            # Add object to planning scene
            self.scene.add_box(
                f"object_{obj.id}",
                pose_stamped,
                size=(obj.dimensions.x, obj.dimensions.y, obj.dimensions.z)
            )
            
            self.get_logger().debug(f"Added object {obj.id} to planning scene")
            
        except Exception as e:
            self.get_logger().error(f"Error adding object to planning scene: {str(e)}")
    
    def pick_command_callback(self, msg):
        """
        Callback for pick commands
        """
        if self.current_state != "idle":
            self.get_logger().warning(f"Cannot execute pick command, current state: {self.current_state}")
            return
        
        self.current_state = "picking"
        self.publish_status("picking")
        
        # Parse command (format: "pick object_id")
        try:
            _, object_id_str = msg.data.split()
            object_id = int(object_id_str)
            
            # Execute pick in a separate thread
            threading.Thread(target=self.execute_pick, args=(object_id,)).start()
            
        except Exception as e:
            self.get_logger().error(f"Error parsing pick command: {str(e)}")
            self.current_state = "idle"
            self.publish_status("error")
    
    def place_command_callback(self, msg):
        """
        Callback for place commands
        """
        if self.current_state != "idle":
            self.get_logger().warning(f"Cannot execute place command, current state: {self.current_state}")
            return
        
        if self.attached_object_id is None:
            self.get_logger().warning("No object attached to end effector")
            self.publish_status("error")
            return
        
        self.current_state = "placing"
        self.publish_status("placing")
        
        # Parse command (format: "place x y z")
        try:
            parts = msg.data.split()
            if len(parts) == 4:
                _, x, y, z = parts
                place_pose = Pose()
                place_pose.position.x = float(x)
                place_pose.position.y = float(y)
                place_pose.position.z = float(z)
                place_pose.orientation.w = 1.0
            elif len(parts) == 2:
                # Format: "place location_name"
                _, location_name = parts
                place_pose = self.get_predefined_place_location(location_name)
            else:
                raise ValueError("Invalid place command format")
            
            # Execute place in a separate thread
            threading.Thread(target=self.execute_place, args=(place_pose,)).start()
            
        except Exception as e:
            self.get_logger().error(f"Error parsing place command: {str(e)}")
            self.current_state = "idle"
            self.publish_status("error")
    
    def get_predefined_place_location(self, location_name):
        """
        Get predefined place location
        """
        # Define some predefined locations
        locations = {
            "bin_a": Pose(
                position=Point(x=0.3, y=0.3, z=0.1),
                orientation=Quaternion(w=1.0)
            ),
            "bin_b": Pose(
                position=Point(x=0.3, y=-0.3, z=0.1),
                orientation=Quaternion(w=1.0)
            ),
            "bin_c": Pose(
                position=Point(x=-0.3, y=0.3, z=0.1),
                orientation=Quaternion(w=1.0)
            ),
            "bin_d": Pose(
                position=Point(x=-0.3, y=-0.3, z=0.1),
                orientation=Quaternion(w=1.0)
            ),
            "home": Pose(
                position=Point(x=0.0, y=0.0, z=0.3),
                orientation=Quaternion(w=1.0)
            )
        }
        
        if location_name in locations:
            return locations[location_name]
        else:
            self.get_logger().error(f"Unknown location: {location_name}")
            raise ValueError(f"Unknown location: {location_name}")
    
    def execute_pick(self, object_id):
        """
        Execute pick operation for specified object
        """
        try:
            with self.object_lock:
                if object_id not in self.detected_objects:
                    self.get_logger().error(f"Object {object_id} not found")
                    self.current_state = "idle"
                    self.publish_status("error")
                    return
                
                target_object = self.detected_objects[object_id]
            
            # Transform object pose to planning frame if needed
            object_pose = target_object.pose
            if target_object.header.frame_id != self.base_link:
                transform = self.tf_buffer.lookup_transform(
                    self.base_link,
                    target_object.header.frame_id,
                    rclpy.time.Time()
                )
                object_pose = do_transform_pose(object_pose, transform)
            
            # Pre-grasp pose (slightly above object)
            pre_grasp_pose = Pose()
            pre_grasp_pose.position.x = object_pose.position.x
            pre_grasp_pose.position.y = object_pose.position.y
            pre_grasp_pose.position.z = object_pose.position.z + self.pre_grasp_distance
            pre_grasp_pose.orientation = object_pose.orientation
            
            # Move to pre-grasp
            self.get_logger().info(f"Moving to pre-grasp position for object {object_id}")
            success = self.move_to_pose(pre_grasp_pose)
            
            if not success:
                self.get_logger().error("Failed to reach pre-grasp position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Move to grasp
            self.get_logger().info(f"Moving to grasp position for object {object_id}")
            success = self.move_to_pose(object_pose)
            
            if not success:
                self.get_logger().error("Failed to reach grasp position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Simulate gripper close
            self.get_logger().info("Closing gripper")
            time.sleep(0.5)  # Simulate gripper closing time
            
            # Attach object to end effector
            self.get_logger().info(f"Attaching object {object_id} to end effector")
            self.attach_object_to_end_effector(object_id)
            
            # Move back to pre-grasp (with object)
            self.get_logger().info("Moving to post-grasp position")
            success = self.move_to_pose(pre_grasp_pose)
            
            if not success:
                self.get_logger().error("Failed to reach post-grasp position")
                # Don't return, try to continue
            
            self.get_logger().info(f"Successfully picked object {object_id}")
            self.current_state = "idle"
            self.publish_status("pick_complete")
            
        except Exception as e:
            self.get_logger().error(f"Error during pick operation: {str(e)}")
            self.current_state = "idle"
            self.publish_status("error")
    
    def execute_place(self, place_pose):
        """
        Execute place operation at specified pose
        """
        try:
            if self.attached_object_id is None:
                self.get_logger().error("No object attached to end effector")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Pre-place pose (slightly above place location)
            pre_place_pose = Pose()
            pre_place_pose.position.x = place_pose.position.x
            pre_place_pose.position.y = place_pose.position.y
            pre_place_pose.position.z = place_pose.position.z + self.post_grasp_distance
            pre_place_pose.orientation = place_pose.orientation
            
            # Move to pre-place
            self.get_logger().info("Moving to pre-place position")
            success = self.move_to_pose(pre_place_pose)
            
            if not success:
                self.get_logger().error("Failed to reach pre-place position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Move to place
            self.get_logger().info("Moving to place position")
            success = self.move_to_pose(place_pose)
            
            if not success:
                self.get_logger().error("Failed to reach place position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Simulate gripper open
            self.get_logger().info("Opening gripper")
            time.sleep(0.5)  # Simulate gripper opening time
            
            # Detach object from end effector
            self.get_logger().info("Detaching object from end effector")
            self.detach_object_from_end_effector()
            
            # Move back to pre-place
            self.get_logger().info("Moving to post-place position")
            success = self.move_to_pose(pre_place_pose)
            
            if not success:
                self.get_logger().error("Failed to reach post-place position")
                # Don't return, try to continue
            
            self.get_logger().info("Successfully placed object")
            self.current_state = "idle"
            self.publish_status("place_complete")
            
        except Exception as e:
            self.get_logger().error(f"Error during place operation: {str(e)}")
            self.current_state = "idle"
            self.publish_status("error")
    
    def move_to_pose(self, target_pose):
        """
        Move the end effector to the target pose
        """
        try:
            # Set the target pose
            self.move_group.set_pose_target(target_pose)
            
            # Plan and execute
            success, plan, planning_time, error_code = self.move_group.plan()
            
            if success:
                self.move_group.execute(plan, wait=True)
            
            # Clear pose targets
            self.move_group.clear_pose_targets()
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"Error moving to pose: {str(e)}")
            return False
    
    def attach_object_to_end_effector(self, object_id):
        """
        Attach object to end effector in planning scene
        """
        try:
            # Attach the object
            self.scene.attach_box(
                self.end_effector_link,
                f"object_{object_id}",
                touch_links=[self.end_effector_link]
            )
            
            # Store the attached object ID
            self.attached_object_id = object_id
            
            self.get_logger().info(f"Object {object_id} attached to end effector")
            
        except Exception as e:
            self.get_logger().error(f"Error attaching object to end effector: {str(e)}")
    
    def detach_object_from_end_effector(self):
        """
        Detach object from end effector in planning scene
        """
        try:
            if self.attached_object_id is not None:
                # Detach the object
                self.scene.remove_attached_object(
                    self.end_effector_link,
                    f"object_{self.attached_object_id}"
                )
                
                # Remove the object from the scene
                self.scene.remove_world_object(f"object_{self.attached_object_id}")
                
                self.get_logger().info(f"Object {self.attached_object_id} detached from end effector")
                
                # Clear the attached object ID
                self.attached_object_id = None
                
        except Exception as e:
            self.get_logger().error(f"Error detaching object from end effector: {str(e)}")
    
    def publish_status(self, status):
        """
        Publish current status
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    pick_and_place_node = PickAndPlaceNode()
    
    # Use multithreaded executor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(pick_and_place_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        pick_and_place_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
