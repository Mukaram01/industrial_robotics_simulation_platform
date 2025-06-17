"""Shared testing utilities."""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock


def _setup_ros_stubs(monkeypatch):
    """Create minimal ROS stubs used across multiple tests."""

    root = Path(__file__).resolve().parents[1]

    # rclpy and node stub
    rclpy_stub = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")

    class DummyLogger:
        def __init__(self):
            self.info = MagicMock()
            self.warning = MagicMock()
            self.warn = MagicMock()
            self.error = MagicMock()

    class DummyNode:
        def __init__(self, *args, **kwargs):
            self.params = {}
            self._logger = DummyLogger()

        def declare_parameter(self, name, value):
            self.params[name] = value
            return types.SimpleNamespace(value=value)

        def declare_parameters(self, ns, params):
            for name, value in params:
                self.params[name] = value

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.params.get(name))

        def create_publisher(self, *args, **kwargs):
            pub = MagicMock()
            pub._topic = args[1] if len(args) > 1 else kwargs.get("topic")
            return pub

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def get_logger(self):
            return self._logger

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    rclpy_stub.init = lambda *a, **kw: None
    rclpy_stub.shutdown = lambda *a, **kw: None

    monkeypatch.setitem(sys.modules, "rclpy", rclpy_stub)
    monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)

    # std_msgs stub
    std_msgs_stub = types.ModuleType("std_msgs")
    std_msgs_stub.msg = types.ModuleType("std_msgs.msg")

    class Msg:
        def __init__(self):
            self.data = None

    std_msgs_stub.msg.String = Msg
    std_msgs_stub.msg.Bool = Msg

    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs_stub)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_stub.msg)

    # sensor_msgs stub
    sensor_stub = types.ModuleType("sensor_msgs")
    sensor_stub.msg = types.ModuleType("sensor_msgs.msg")
    sensor_stub.msg.Image = Msg

    class JS:
        def __init__(self):
            self.name = []
            self.position = []
            self.velocity = []

    sensor_stub.msg.JointState = JS
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_stub)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_stub.msg)

    # cv_bridge and cv2 stubs
    cv_bridge_stub = types.ModuleType("cv_bridge")

    class CvBridge:
        def imgmsg_to_cv2(self, *a, **k):
            return None

    cv_bridge_stub.CvBridge = CvBridge
    monkeypatch.setitem(sys.modules, "cv_bridge", cv_bridge_stub)

    cv2_stub = types.ModuleType("cv2")
    cv2_stub.imencode = lambda *a, **k: (True, b"")
    monkeypatch.setitem(sys.modules, "cv2", cv2_stub)

    # flask stub uses real flask as tests rely on it
    import flask

    monkeypatch.setitem(sys.modules, "flask", flask)

    # ament index stub
    ament_stub = types.ModuleType("ament_index_python")
    ament_stub.packages = types.ModuleType("ament_index_python.packages")

    def get_pkg_share(_pkg):
        return str(root / "src" / "web_interface_frontend")

    ament_stub.packages.get_package_share_directory = get_pkg_share

    monkeypatch.setitem(sys.modules, "ament_index_python", ament_stub)
    monkeypatch.setitem(
        sys.modules,
        "ament_index_python.packages",
        ament_stub.packages,
    )

