#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
from ament_index_python.packages import get_package_share_directory

class AdvancedPerceptionNode(Node):
    """
    Main node for advanced perception package
    """
    def __init__(self):
        super().__init__('advanced_perception_node')
        
        self.get_logger().info('Advanced Perception Node initialized')
        self.get_logger().info('This node serves as a central coordinator for advanced perception components')

def main(args=None):
    rclpy.init(args=args)

    advanced_perception_node = AdvancedPerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(advanced_perception_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        advanced_perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
