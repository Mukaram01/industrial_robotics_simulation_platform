"""UR5 robot interface helpers."""

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from typing import Dict


class UR5Interface:
    """Interface for sending commands to a UR5 robot."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.command_pub = node.create_publisher(String, "/simulation/command", 10)
        self.status_sub = node.create_subscription(
            String, "/simulation/status", self._status_callback, 10
        )
        self.last_status = ""

    def _status_callback(self, msg: String) -> None:
        self.last_status = msg.data

    def send_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def get_status(self) -> str:
        return self.last_status

    @staticmethod
    def convert_joint_state(msg: JointState) -> Dict[str, float]:
        return {n: p for n, p in zip(msg.name, msg.position)}

