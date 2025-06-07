#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WebInterfaceNode(Node):
    """
    Main node for web interface package
    """
    def __init__(self):
        super().__init__('web_interface_node')
        
        self.get_logger().info('Web Interface Node initialized')
        self.get_logger().info('This node serves as a central coordinator for web interface components')

def main(args=None):
    rclpy.init(args=args)
    
    web_interface_node = WebInterfaceNode()
    
    try:
        rclpy.spin(web_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
