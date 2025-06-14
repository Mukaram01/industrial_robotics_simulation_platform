import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))


def _setup_ros_stubs(monkeypatch):
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class DummyNode:
        def __init__(self, *args, **kwargs):
            self.params = {}
            self._logger = types.SimpleNamespace(
                info=lambda *a, **k: None,
                warn=lambda *a, **k: None,
                warning=lambda *a, **k: None,
                error=lambda *a, **k: None,
            )

        def declare_parameter(self, name, value):
            self.params[name] = value

        def get_parameter(self, name):
            class P:
                def __init__(self, value):
                    self.value = value
            return P(self.params.get(name))

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

    class Msg:
        def __init__(self):
            self.data = False

    std_msgs_stub.msg.Bool = Msg
    std_msgs_stub.msg.String = Msg
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_stub)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_stub.msg)


def test_emergency_stop_trigger(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.safety_monitor_node', None)
    from simulation_core import safety_monitor_node as smn

    node = smn.SafetyMonitorNode()
    node.emergency_stop_pub.publish.reset_mock()
    node.trigger_emergency_stop('manual')

    assert node.emergency_stop_active is True
    assert node.emergency_stop_pub.publish.called
    msg = node.emergency_stop_pub.publish.call_args[0][0]
    assert msg.data is True
    assert node.emergency_stop_pub._topic == '/safety/emergency_stop'


def test_emergency_stop_reset(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.safety_monitor_node', None)
    from simulation_core import safety_monitor_node as smn

    node = smn.SafetyMonitorNode()
    node.emergency_stop_active = True
    node.safety_violations = []
    node.emergency_stop_pub.publish.reset_mock()
    node.reset_emergency_stop()

    assert node.emergency_stop_active is False
    assert node.emergency_stop_pub.publish.called
    msg = node.emergency_stop_pub.publish.call_args[0][0]
    assert msg.data is False
    assert node.emergency_stop_pub._topic == '/safety/emergency_stop'
