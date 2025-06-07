#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

class SortingDemoNode(Node):
    """
    Node for demonstrating the sorting application with the Delta robot
    """
    def __init__(self):
        super().__init__('sorting_demo_node')
        
        # Declare parameters
        self.declare_parameter('pick_command_topic', '/fmm/pick_command')
        self.declare_parameter('place_command_topic', '/fmm/place_command')
        self.declare_parameter('status_topic', '/fmm/status')
        self.declare_parameter('sorting_locations', {
            'red': [0.3, 0.3, 0.1],
            'green': [0.3, -0.3, 0.1],
            'blue': [-0.3, 0.3, 0.1],
            'unknown': [-0.3, -0.3, 0.1]
        })
        
        # Get parameters
        self.pick_command_topic = self.get_parameter('pick_command_topic').value
        self.place_command_topic = self.get_parameter('place_command_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.sorting_locations = self.get_parameter('sorting_locations').value
        
        # Create publishers
        self.pick_command_pub = self.create_publisher(
            String,
            self.pick_command_topic,
            10
        )
        
        self.place_command_pub = self.create_publisher(
            String,
            self.place_command_topic,
            10
        )
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            String,
            self.status_topic,
            self.status_callback,
            10
        )
        
        # Initialize state
        self.current_status = "unknown"
        self.current_object_id = None
        self.current_object_class = None
        self.demo_running = False
        self.demo_thread = None
        
        # Create timer for demo control
        self.demo_timer = self.create_timer(1.0, self.demo_control_callback)
        
        self.get_logger().info('Sorting Demo Node initialized')
    
    def status_callback(self, msg):
        """
        Callback for robot status messages
        """
        self.current_status = msg.data
        self.get_logger().info(f'Received status: {self.current_status}')
    
    def demo_control_callback(self):
        """
        Timer callback for demo control
        """
        if not self.demo_running and self.current_status == "ready":
            self.get_logger().info('Starting sorting demo')
            self.demo_running = True
            self.demo_thread = threading.Thread(target=self.run_sorting_demo)
            self.demo_thread.start()
    
    def run_sorting_demo(self):
        """
        Run the sorting demo
        """
        try:
            # List of objects to sort (simulated)
            objects_to_sort = [
                {"id": 1, "class": "red"},
                {"id": 2, "class": "green"},
                {"id": 3, "class": "blue"},
                {"id": 4, "class": "unknown"}
            ]
            
            for obj in objects_to_sort:
                # Wait for robot to be ready
                while self.current_status != "ready" and self.current_status != "pick_complete":
                    time.sleep(0.1)
                
                # Pick the object
                self.current_object_id = obj["id"]
                self.current_object_class = obj["class"]
                self.get_logger().info(f'Picking object {self.current_object_id} (class: {self.current_object_class})')
                
                pick_msg = String()
                pick_msg.data = f'pick {self.current_object_id}'
                self.pick_command_pub.publish(pick_msg)
                
                # Wait for pick to complete
                while self.current_status != "pick_complete":
                    time.sleep(0.1)
                    if self.current_status == "error":
                        self.get_logger().error(f'Error picking object {self.current_object_id}')
                        break
                
                if self.current_status == "error":
                    continue
                
                # Wait a moment
                time.sleep(1.0)
                
                # Place the object in the appropriate bin
                if self.current_object_class in self.sorting_locations:
                    location = self.sorting_locations[self.current_object_class]
                else:
                    location = self.sorting_locations["unknown"]
                
                self.get_logger().info(f'Placing object {self.current_object_id} at location {location}')
                
                place_msg = String()
                place_msg.data = f'place {location[0]} {location[1]} {location[2]}'
                self.place_command_pub.publish(place_msg)
                
                # Wait for place to complete
                while self.current_status != "place_complete":
                    time.sleep(0.1)
                    if self.current_status == "error":
                        self.get_logger().error(f'Error placing object {self.current_object_id}')
                        break
                
                # Wait a moment before next object
                time.sleep(1.0)
            
            self.get_logger().info('Sorting demo completed')
            
        except Exception as e:
            self.get_logger().error(f'Error in sorting demo: {str(e)}')
        finally:
            self.demo_running = False

def main(args=None):
    rclpy.init(args=args)
    
    sorting_demo_node = SortingDemoNode()
    
    try:
        rclpy.spin(sorting_demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        sorting_demo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
