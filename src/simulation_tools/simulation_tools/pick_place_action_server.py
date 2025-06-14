#!/usr/bin/env python3

"""Example action server for a pick and place operation."""

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci


class PickPlaceActionServer(Node):
    """Demonstration action server using the Fibonacci action."""

    def __init__(self):
        super().__init__('pick_place_action_server')
        self._server = ActionServer(
            self,
            Fibonacci,
            'demo_pick_place',
            self.execute_callback,
        )

    async def execute_callback(self, goal_handle):
        order = goal_handle.request.order
        sequence = [0, 1]
        feedback = Fibonacci.Feedback()
        feedback.sequence = sequence.copy()
        goal_handle.publish_feedback(feedback)

        for _ in range(2, order):
            sequence.append(sequence[-1] + sequence[-2])
            feedback.sequence = sequence.copy()
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = PickPlaceActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
