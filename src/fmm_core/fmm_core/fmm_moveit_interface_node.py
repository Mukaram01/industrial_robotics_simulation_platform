#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from apm_msgs.msg import DetectedObject, DetectedObjectArray
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import moveit_commander
import sys
import time

class FMMMoveitInterfaceNode(Node):
    """
    Node for interfacing with MoveIt2 for Delta robot control
    """
    def __init__(self):
        super().__init__('fmm_moveit_interface_node')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'delta_robot')
        self.declare_parameter('planning_group', 'delta_arm')
        self.declare_parameter('end_effector_link', 'end_effector')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('detected_objects_topic', '/apm/detection/objects')
        self.declare_parameter('pick_command_topic', '/fmm/pick_command')
        self.declare_parameter('place_command_topic', '/fmm/place_command')
        self.declare_parameter('status_topic', '/fmm/status')
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_link = self.get_parameter('base_link').value
        self.detected_objects_topic = self.get_parameter('detected_objects_topic').value
        self.pick_command_topic = self.get_parameter('pick_command_topic').value
        self.place_command_topic = self.get_parameter('place_command_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        
        # Create callback groups
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()
        
        # Initialize MoveIt2 Commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)
        
        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
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
        
        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk',
            callback_group=self.service_callback_group
        )
        
        # Initialize object tracking
        self.detected_objects = {}
        self.object_lock = threading.Lock()
        self.current_state = "idle"
        
        # Wait for services
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
        
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting...')
        
        self.get_logger().info('FMM MoveIt Interface Node initialized')
        self.publish_status("ready")
    
    def detected_objects_callback(self, msg):
        """
        Callback for processing detected objects
        """
        with self.object_lock:
            # Update object dictionary
            for obj in msg.objects:
                self.detected_objects[obj.id] = obj
    
    def pick_command_callback(self, msg):
        """
        Callback for pick commands
        """
        if self.current_state != "idle":
            self.get_logger().warn(f"Cannot execute pick command, current state: {self.current_state}")
            return
        
        self.current_state = "picking"
        self.publish_status("picking")
        
        # Parse command (format: "pick object_id")
        try:
            _, object_id = msg.data.split()
            object_id = int(object_id)
            
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
            self.get_logger().warn(f"Cannot execute place command, current state: {self.current_state}")
            return
        
        self.current_state = "placing"
        self.publish_status("placing")
        
        # Parse command (format: "place x y z")
        try:
            _, x, y, z = msg.data.split()
            place_pose = Pose()
            place_pose.position.x = float(x)
            place_pose.position.y = float(y)
            place_pose.position.z = float(z)
            place_pose.orientation.w = 1.0
            
            # Execute place in a separate thread
            threading.Thread(target=self.execute_place, args=(place_pose,)).start()
            
        except Exception as e:
            self.get_logger().error(f"Error parsing place command: {str(e)}")
            self.current_state = "idle"
            self.publish_status("error")
    
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
            pre_grasp_pose.position.z = object_pose.position.z + 0.1  # 10cm above
            pre_grasp_pose.orientation = object_pose.orientation
            
            # Move to pre-grasp
            self.get_logger().info(f"Moving to pre-grasp position for object {object_id}")
            self.move_group.set_pose_target(pre_grasp_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if not success:
                self.get_logger().error("Failed to reach pre-grasp position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Move to grasp
            self.get_logger().info(f"Moving to grasp position for object {object_id}")
            self.move_group.set_pose_target(object_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
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
            # Code to attach object in MoveIt scene would go here
            
            # Move back to pre-grasp (with object)
            self.get_logger().info("Moving to post-grasp position")
            self.move_group.set_pose_target(pre_grasp_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
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
            # Pre-place pose (slightly above place location)
            pre_place_pose = Pose()
            pre_place_pose.position.x = place_pose.position.x
            pre_place_pose.position.y = place_pose.position.y
            pre_place_pose.position.z = place_pose.position.z + 0.1  # 10cm above
            pre_place_pose.orientation = place_pose.orientation
            
            # Move to pre-place
            self.get_logger().info("Moving to pre-place position")
            self.move_group.set_pose_target(pre_place_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if not success:
                self.get_logger().error("Failed to reach pre-place position")
                self.current_state = "idle"
                self.publish_status("error")
                return
            
            # Move to place
            self.get_logger().info("Moving to place position")
            self.move_group.set_pose_target(place_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
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
            # Code to detach object in MoveIt scene would go here
            
            # Move back to pre-place
            self.get_logger().info("Moving to post-place position")
            self.move_group.set_pose_target(pre_place_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
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
    
    def publish_status(self, status):
        """
        Publish current status
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    moveit_interface = FMMMoveitInterfaceNode()
    
    # Use multithreaded executor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(moveit_interface)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        moveit_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
