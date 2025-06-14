import sys
import types
from unittest.mock import MagicMock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'perception_nodes'))


def _setup_stubs(monkeypatch):
    """Create minimal stubs so CameraSimulatorNode can be imported."""
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    exec_mod = types.ModuleType('rclpy.executors')

    class Logger:
        def __init__(self):
            self.info = MagicMock()
            self.warn = MagicMock()
            self.warning = MagicMock()
            self.error = MagicMock()

    class DummyNode:
        def __init__(self, *a, **k):
            self.params = {}
            self._logger = Logger()

        def declare_parameters(self, ns, params):
            for name, value in params:
                self.params[name] = 0.0 if name == 'frame_rate' else value

        class P:
            def __init__(self, value):
                self.value = value

        def get_parameter(self, name):
            return self.P(self.params.get(name))

        def create_publisher(self, *a, **k):
            return MagicMock()

        def create_subscription(self, *a, **k):
            return MagicMock()

        def create_timer(self, *a, **k):
            return MagicMock()

        def get_logger(self):
            return self._logger

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    exec_mod.MultiThreadedExecutor = MagicMock()
    rclpy_stub.executors = exec_mod
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', exec_mod)

    sensor_stub = types.ModuleType('sensor_msgs')
    sensor_stub.msg = types.ModuleType('sensor_msgs.msg')

    class Header:
        def __init__(self):
            self.frame_id = ''

    class CameraInfo:
        def __init__(self):
            self.header = Header()
            self.width = 0
            self.height = 0
            self.k = []
            self.p = []
            self.distortion_model = ''
            self.d = []

    sensor_stub.msg.Image = object
    sensor_stub.msg.CameraInfo = CameraInfo
    sensor_stub.msg.CompressedImage = object
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_stub)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_stub.msg)

    std_msgs_stub = types.ModuleType('std_msgs')
    std_msgs_stub.msg = types.ModuleType('std_msgs.msg')
    std_msgs_stub.msg.String = object
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_stub)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_stub.msg)

    cv_bridge_stub = types.ModuleType('cv_bridge')
    cv_bridge_stub.CvBridge = MagicMock()
    monkeypatch.setitem(sys.modules, 'cv_bridge', cv_bridge_stub)

    cv2_stub = types.ModuleType('cv2')
    cv2_stub.imencode = lambda *a, **k: (True, b'')
    cv2_stub.line = lambda *a, **k: None
    cv2_stub.rectangle = lambda *a, **k: None
    cv2_stub.circle = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'cv2', cv2_stub)


def test_frame_rate_zero_warns(monkeypatch):
    _setup_stubs(monkeypatch)
    sys.modules.pop('perception_nodes.synthetic_camera_node', None)
    from perception_nodes import synthetic_camera_node as csn

    node = csn.CameraSimulatorNode()
    logger = node.get_logger()
    logger.warning.assert_called_once()
    assert 'Invalid frame_rate' in logger.warning.call_args[0][0]
    assert node.frame_rate == 30.0
