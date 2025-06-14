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

    class DummyLogger:
        def __init__(self):
            self.info = MagicMock()
            self.warning = MagicMock()
            self.error = MagicMock()

    class DummyNode:
        def __init__(self, *a, **k):
            self.params = {}
            self._logger = DummyLogger()

        def declare_parameter(self, name, value):
            self.params[name] = value
            return self.Param(value)

        class Param:
            def __init__(self, value):
                self.value = value

        def get_parameter(self, name):
            return self.Param(self.params.get(name))

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
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)

    std_msgs_stub = types.ModuleType('std_msgs')
    std_msgs_stub.msg = types.ModuleType('std_msgs.msg')

    class Msg:
        def __init__(self):
            self.data = ''

    std_msgs_stub.msg.String = Msg
    std_msgs_stub.msg.Bool = Msg
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_stub)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_stub.msg)


def test_timer_logs_once_per_interval(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.system_test_node', None)
    from simulation_core import system_test_node as stn

    node = stn.SystemTestNode()
    logger = node.get_logger()
    logger.info.reset_mock()

    node.test_running = True
    node.start_time = 0.0
    node.test_duration = 100.0
    node._last_log_time = 0.0

    times = [1, 10, 10.5, 20]

    def fake_time():
        return times.pop(0)

    monkeypatch.setattr(stn.time, 'time', fake_time)

    node.timer_callback()  # t=1, no log
    node.timer_callback()  # t=10, log
    node.timer_callback()  # t=10.5, no log
    node.timer_callback()  # t=20, log

    assert logger.info.call_count == 2
