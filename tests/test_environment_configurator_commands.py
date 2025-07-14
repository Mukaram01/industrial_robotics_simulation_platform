import sys
from pathlib import Path
from test_utils import _setup_ros_stubs


ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "src"))
sys.path.append(str(ROOT / "src" / "simulation_core"))


def _msg(data: str):
    from std_msgs.msg import String
    m = String()
    m.data = data
    return m


def test_start_stop_recording(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop("simulation_core.environment_configurator_node", None)
    sys.modules.pop("simulation_core", None)
    from simulation_core import environment_configurator_node as ec

    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, "_load_robot_models", lambda self: None)

    procs = []

    class DummyProc:
        def __init__(self):
            self.terminated = False
            self.waited = False

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=None):
            self.waited = True

    def dummy_popen(cmd, *a, **k):
        proc = DummyProc()
        procs.append((cmd, proc))
        return proc

    monkeypatch.setattr(ec.subprocess, "Popen", dummy_popen)
    monkeypatch.setattr(ec.time, "strftime", lambda fmt: "bagtest")

    bag_dir = Path("data/recordings")
    bag_dir.mkdir(parents=True, exist_ok=True)

    node = ec.EnvironmentConfiguratorNode()
    node.command_callback(_msg("start_recording"))

    assert procs
    cmd, proc = procs[0]
    assert cmd[:3] == ["ros2", "bag", "record"]
    assert Path(cmd[-1]).parent == bag_dir
    assert node._record_proc is proc

    node.command_callback(_msg("stop_recording"))

    assert proc.terminated and proc.waited
    assert node._record_proc is None


def test_start_stop_playback(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop("simulation_core.environment_configurator_node", None)
    sys.modules.pop("simulation_core", None)
    from simulation_core import environment_configurator_node as ec

    monkeypatch.setattr(ec.EnvironmentConfiguratorNode, "_load_robot_models", lambda self: None)

    procs = []

    class DummyProc:
        def __init__(self):
            self.terminated = False
            self.waited = False

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=None):
            self.waited = True

    def dummy_popen(cmd, *a, **k):
        proc = DummyProc()
        procs.append((cmd, proc))
        return proc

    monkeypatch.setattr(ec.subprocess, "Popen", dummy_popen)

    bag_dir = Path("data/recordings")
    bag = bag_dir / "demo"
    bag.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(ec.os.path, "isdir", lambda d: True if d == str(bag_dir) else False)
    monkeypatch.setattr(ec.os, "listdir", lambda d: [bag.name] if d == str(bag_dir) else [])

    node = ec.EnvironmentConfiguratorNode()
    node.command_callback(_msg("start_playback"))

    assert procs
    cmd, proc = procs[0]
    assert cmd[:3] == ["ros2", "bag", "play"]
    assert cmd[-1] == str(bag)
    assert node._play_proc is proc

    node.command_callback(_msg("stop_playback"))

    assert proc.terminated and proc.waited
    assert node._play_proc is None

