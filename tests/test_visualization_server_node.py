import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))


def _setup_stubs(monkeypatch):
    """Create minimal stubs so VisualizationServerNode can be imported."""
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class Logger:
        def __init__(self):
            self.info = MagicMock()
            self.warn = MagicMock()
            self.warning = MagicMock()
            self.error = MagicMock()

    class DummyNode:
        def __init__(self, *args, **kwargs):
            self.params = {}
            self._logger = Logger()

        def declare_parameter(self, name, value):
            self.params[name] = 0.0 if name == 'visualization_rate' else value

        class P:
            def __init__(self, value):
                self.value = value

        def get_parameter(self, name):
            return self.P(self.params.get(name))

        def create_publisher(self, *args, **kwargs):
            pub = MagicMock()
            pub._topic = args[1] if len(args) > 1 else kwargs.get('topic')
            return pub

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_timer(self, *args, **kwargs):
            return MagicMock()

        def get_logger(self):
            return self._logger

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)

    std_msgs_stub = types.ModuleType('std_msgs')
    std_msgs_stub.msg = types.ModuleType('std_msgs.msg')
    std_msgs_stub.msg.String = object
    std_msgs_stub.msg.Bool = object
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_stub)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_stub.msg)

    sensor_stub = types.ModuleType('sensor_msgs')
    sensor_stub.msg = types.ModuleType('sensor_msgs.msg')
    sensor_stub.msg.Image = object
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_stub)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_stub.msg)

    cv_bridge_stub = types.ModuleType('cv_bridge')
    cv_bridge_stub.CvBridge = MagicMock()
    monkeypatch.setitem(sys.modules, 'cv_bridge', cv_bridge_stub)

    cv2_stub = types.ModuleType('cv2')
    cv2_stub.imencode = lambda *a, **k: (True, b'')
    monkeypatch.setitem(sys.modules, 'cv2', cv2_stub)


def test_visualization_rate_zero_warns(monkeypatch):
    _setup_stubs(monkeypatch)
    sys.modules.pop('web_interface_backend.visualization_server_node', None)
    from web_interface_backend import visualization_server_node as vsn

    node = vsn.VisualizationServerNode()
    logger = node.get_logger()
    logger.warning.assert_called_once()
    assert 'Invalid visualization_rate' in logger.warning.call_args[0][0]
    assert node.visualization_rate == 10.0
