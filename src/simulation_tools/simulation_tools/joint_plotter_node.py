#!/usr/bin/env python3
"""ROS2 node plotting joint state positions in real time."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from matplotlib import pyplot as plt


class JointPlotterNode(Node):
    """Subscribe to `/joint_states` and plot joint positions."""

    def __init__(self) -> None:
        super().__init__("joint_plotter_node")

        self.declare_parameter("joint_states_topic", "/joint_states")
        topic = self.get_parameter("joint_states_topic").value

        self._lock = threading.Lock()
        self._names: list[str] = []
        self._positions: list[list[float]] = []
        self._times: list[float] = []

        self.create_subscription(JointState, topic, self.joint_callback, 10)

        plt.ion()
        self._fig, self._ax = plt.subplots()
        self._timer = self.create_timer(0.5, self.update_plot)

        self.get_logger().info("Joint plotter node initialized")

    def joint_callback(self, msg: JointState) -> None:
        with self._lock:
            if not self._names:
                self._names = list(msg.name)
            self._times.append(self.get_clock().now().nanoseconds / 1e9)
            self._positions.append(list(msg.position))

    def update_plot(self) -> None:
        with self._lock:
            if not self._times or not self._names:
                return
            times = self._times
            data = list(zip(*self._positions))
            self._ax.clear()
            for idx, series in enumerate(data):
                label = self._names[idx] if idx < len(self._names) else f"joint{idx}"
                self._ax.plot(times, series, label=label)
            self._ax.set_xlabel("Time (s)")
            self._ax.set_ylabel("Position (rad)")
            self._ax.legend()
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JointPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
