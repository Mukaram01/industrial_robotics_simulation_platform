#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from . import registry


class RobotControlNode(Node):
    """Translate high level commands into robot specific topics."""

    def __init__(self):
        super().__init__('robot_control_node')

        self.declare_parameter('jog_topic', '/robot/jog')
        self.declare_parameter('waypoint_topic', '/robot/waypoint')
        self.declare_parameter('sequence_topic', '/robot/execute_sequence')
        self.declare_parameter('controller_type', 'basic')

        self.jog_topic = self.get_parameter('jog_topic').value
        self.waypoint_topic = self.get_parameter('waypoint_topic').value
        self.sequence_topic = self.get_parameter('sequence_topic').value
        controller_name = self.get_parameter('controller_type').value
        controller_cls = registry.get_controller(controller_name)
        if controller_cls is None:
            self.get_logger().warning(
                f'Controller {controller_name} not found, using basic controller'
            )
            controller_cls = registry.get_controller('basic')
        self.controller = controller_cls()

        self.jog_pub = self.create_publisher(String, self.jog_topic, 10)
        self.waypoint_pub = self.create_publisher(String, self.waypoint_topic, 10)
        self.sequence_pub = self.create_publisher(String, self.sequence_topic, 10)

        self.command_sub = self.create_subscription(
            String,
            '/simulation/command',
            self.command_callback,
            10,
        )

        self.get_logger().info('Robot control node initialized')

    def command_callback(self, msg: String):
        parts = msg.data.split()
        if not parts:
            return

        cmd = parts[0]
        if cmd == 'jog' and len(parts) == 3:
            out = String()
            out.data = f"{parts[1]} {parts[2]}"
            self.jog_pub.publish(out)
        elif cmd in ('record_waypoint', 'clear_waypoints'):
            out = String()
            out.data = cmd
            self.waypoint_pub.publish(out)
        elif cmd == 'execute_sequence':
            out = String()
            out.data = 'execute'
            self.sequence_pub.publish(out)
        else:
            self.get_logger().warning(f'Unsupported command: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
