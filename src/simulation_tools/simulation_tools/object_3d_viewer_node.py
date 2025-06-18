#!/usr/bin/env python3
"""ROS2 node visualizing detected objects in 3D using Matplotlib."""

import threading

import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from apm_msgs.msg import DetectedObjectArray


class Object3DViewerNode(Node):
    """Display DetectedObjectArray messages using Matplotlib."""

    def __init__(self) -> None:
        super().__init__('object_3d_viewer_node')

        self.declare_parameter('objects_topic', '/apm/detection/objects')
        self.objects_topic = self.get_parameter('objects_topic').value

        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            self.objects_topic,
            self.objects_callback,
            10,
        )

        self.objects = []
        self.lock = threading.Lock()

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.timer = self.create_timer(0.5, self.update_plot)

        self.get_logger().info('Object 3D viewer node initialized')

    def objects_callback(self, msg: DetectedObjectArray) -> None:
        """Store latest object positions."""
        positions = [
            (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
            for obj in msg.objects
        ]
        with self.lock:
            self.objects = positions

    def update_plot(self) -> None:
        """Refresh Matplotlib scatter plot."""
        with self.lock:
            xs = [p[0] for p in self.objects]
            ys = [p[1] for p in self.objects]
            zs = [p[2] for p in self.objects]
        self.ax.clear()
        if xs:
            self.ax.scatter(xs, ys, zs, c='r', marker='o')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Detected Objects')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Object3DViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
