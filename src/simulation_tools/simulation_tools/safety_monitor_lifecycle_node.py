#!/usr/bin/env python3

"""A minimal safety monitor implemented as a ROS2 Lifecycle node."""

import asyncio
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter, SetParametersResult
from std_msgs.msg import Bool


class SafetyMonitorLifecycleNode(LifecycleNode):
    """Example lifecycle node demonstrating managed states."""

    def __init__(self):
        super().__init__('safety_monitor_lifecycle_node')
        self.check_interval = 0.1
        self.timer = None
        self.emergency_stop_pub = None

    # Lifecycle callbacks -------------------------------------------------
    def on_configure(self, state):
        self.declare_parameter('check_interval', 0.1)
        self.check_interval = self.get_parameter('check_interval').value
        self.add_on_set_parameters_callback(self._param_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.emergency_stop_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10)
        self.timer = self.create_timer(self.check_interval, self._safety_check)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.emergency_stop_pub:
            self.emergency_stop_pub.destroy()
            self.emergency_stop_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    # Internal helpers ----------------------------------------------------
    def _safety_check(self):
        # Placeholder for actual safety logic
        pass

    def _param_callback(self, params):
        for p in params:
            if p.name == 'check_interval' and p.type_ == Parameter.Type.DOUBLE:
                self.check_interval = float(p.value)
                if self.timer:
                    self.timer.cancel()
                    self.timer = self.create_timer(self.check_interval, self._safety_check)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorLifecycleNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
