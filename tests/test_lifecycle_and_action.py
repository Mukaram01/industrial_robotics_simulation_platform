import sys
import types
import asyncio
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))


def _setup_lifecycle_stubs(monkeypatch):
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    lifecycle_mod = types.ModuleType('rclpy.lifecycle')
    param_mod = types.ModuleType('rclpy.parameter')

    class DummyNode:
        def __init__(self, name):
            self.name = name
            self.params = {}
            self._logger = MagicMock()

        def declare_parameter(self, name, value):
            self.params[name] = value

        def get_parameter(self, name):

            class P:
                def __init__(self, v):
                    self.value = v
                    self.type_ = param_mod.Parameter.Type.DOUBLE

            return P(self.params.get(name))

        def add_on_set_parameters_callback(self, cb):
            self._param_cb = cb

        def create_publisher(self, *a, **k):
            pub = MagicMock()
            pub.destroy = MagicMock()
            return pub

        def create_timer(self, *a, **k):
            timer = MagicMock()
            timer.cancel = MagicMock()
            return timer

        def destroy_timer(self, timer):
            pass

        def get_logger(self):
            return self._logger

    node_mod.Node = DummyNode
    lifecycle_mod.LifecycleNode = DummyNode
    lifecycle_mod.TransitionCallbackReturn = types.SimpleNamespace(SUCCESS=1)

    class ParamType:
        DOUBLE = 1

    class Parameter:
        Type = ParamType

    class SetResult:
        def __init__(self, successful=True):
            self.successful = successful
    param_mod.Parameter = Parameter
    param_mod.SetParametersResult = SetResult

    rclpy_stub.node = node_mod
    rclpy_stub.lifecycle = lifecycle_mod
    rclpy_stub.parameter = param_mod
    rclpy_stub.executors = types.ModuleType('rclpy.executors')
    rclpy_stub.executors.SingleThreadedExecutor = MagicMock()
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.lifecycle', lifecycle_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.parameter', param_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', rclpy_stub.executors)

    std_msgs_stub = types.ModuleType('std_msgs')
    std_msgs_stub.msg = types.ModuleType('std_msgs.msg')
    std_msgs_stub.msg.Bool = MagicMock
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_stub)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_stub.msg)


def test_lifecycle_configure(monkeypatch):
    _setup_lifecycle_stubs(monkeypatch)
    sys.modules.pop('simulation_tools.simulation_tools.safety_monitor_lifecycle_node', None)
    from simulation_tools.simulation_tools import safety_monitor_lifecycle_node as sm

    node = sm.SafetyMonitorLifecycleNode()
    result = node.on_configure(None)
    assert result == 1
    assert node.check_interval == 0.1


def _setup_action_stubs(monkeypatch):
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    action_mod = types.ModuleType('rclpy.action')

    class DummyNode:
        def __init__(self, name):
            self.name = name
            self._logger = MagicMock()
        def get_logger(self):
            return self._logger
    node_mod.Node = DummyNode

    class DummyActionServer:
        def __init__(self, node, action_type, name, cb):
            self.callback = cb
    action_mod.ActionServer = DummyActionServer

    rclpy_stub.node = node_mod
    rclpy_stub.action = action_mod
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.action', action_mod)

    ex_stub = types.ModuleType('example_interfaces')
    ex_stub.action = types.ModuleType('example_interfaces.action')

    class Fibonacci:
        class Feedback:
            def __init__(self):
                self.sequence = []

        class Result:
            def __init__(self):
                self.sequence = []

        class Goal:
            def __init__(self, order=1):
                self.order = order
    ex_stub.action.Fibonacci = Fibonacci
    monkeypatch.setitem(sys.modules, 'example_interfaces', ex_stub)
    monkeypatch.setitem(sys.modules, 'example_interfaces.action', ex_stub.action)

    return Fibonacci


def test_action_execute(monkeypatch):
    _setup_action_stubs(monkeypatch)
    sys.modules.pop('simulation_tools.simulation_tools.pick_place_action_server', None)
    from simulation_tools.simulation_tools import pick_place_action_server as pas

    node = pas.PickPlaceActionServer()

    class DummyGoalHandle:
        def __init__(self, order):
            self.request = types.SimpleNamespace(order=order)
            self._feedback = []
            self._canceled = False
            self._succeeded = False
            self.is_cancel_requested = False

        def publish_feedback(self, fb):
            self._feedback.append(list(fb.sequence))

        def canceled(self):
            self._canceled = True

        def succeed(self):
            self._succeeded = True

    gh = DummyGoalHandle(5)
    result = asyncio.run(node.execute_callback(gh))
    assert gh._succeeded is True
    assert result.sequence[-1] == 3
