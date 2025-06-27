import sys
from pathlib import Path
from unittest.mock import MagicMock
from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "src"))
sys.path.append(str(ROOT / "src" / "simulation_core"))


def test_shutdown_cancels_timers(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    timers = []

    def create_timer(self, *args, **kwargs):
        timer = MagicMock()
        timers.append(timer)
        return timer

    sys.modules['rclpy.node'].Node.create_timer = create_timer
    sys.modules.pop('simulation_core.environment_configurator_node', None)
    sys.modules.pop('simulation_core', None)
    from simulation_core import environment_configurator_node as ec

    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, '_load_robot_models', lambda self: None)
    node = ec.EnvironmentConfiguratorNode()
    node.shutdown()

    assert timers
    assert all(t.cancel.called for t in timers)
