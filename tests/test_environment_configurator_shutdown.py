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


def test_shutdown_removes_temp_urdfs(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('simulation_core.environment_configurator_node', None)
    sys.modules.pop('simulation_core', None)
    from simulation_core import environment_configurator_node as ec

    # Avoid spawning external processes
    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, '_launch_process', lambda *a, **k: None)

    original = ec.EnvironmentConfiguratorNode._load_robot_models
    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, '_load_robot_models', lambda self: None)
    node = ec.EnvironmentConfiguratorNode()
    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, '_load_robot_models', original)

    class DummyResult:
        def toxml(self):
            return '<robot/>'

    import types
    monkeypatch.setattr(ec, 'xacro', types.SimpleNamespace(process_file=lambda p: DummyResult()))
    urdf_mod = types.SimpleNamespace(URDF=types.SimpleNamespace(
        from_xml_string=lambda *a, **k: None,
        from_xml_file=lambda *a, **k: None,
    ))
    sys.modules['urdf_parser_py.urdf'] = urdf_mod

    node.environment_config = {
        'robots': [{'id': 'r1', 'model_file': 'robot.urdf.xacro'}]
    }
    node._tmp_files.clear()
    node._load_robot_models()

    tmp_paths = list(node._tmp_files)
    assert tmp_paths
    assert all(Path(p).exists() for p in tmp_paths)

    node.shutdown()

    assert all(not Path(p).exists() for p in tmp_paths)
