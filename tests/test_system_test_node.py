import sys
from pathlib import Path
from unittest.mock import MagicMock

from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))

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
