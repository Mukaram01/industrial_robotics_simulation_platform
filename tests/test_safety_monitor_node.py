import sys
from pathlib import Path

from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))

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


def test_collision_detection(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.safety_monitor_node', None)
    from simulation_core import safety_monitor_node as smn

    node = smn.SafetyMonitorNode()
    node.environment_config = {
        'objects': [
            {'id': 'a', 'type': 'box', 'position': [0.0, 0.0, 0.0], 'dimensions': [1, 1, 1]},
            {'id': 'b', 'type': 'box', 'position': [0.4, 0.0, 0.0], 'dimensions': [1, 1, 1]},
        ]
    }
    violations = node.check_collisions()
    assert violations
    assert violations[0]['type'] == 'collision'


def test_zone_violation_warns(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.safety_monitor_node', None)
    from simulation_core import safety_monitor_node as smn

    node = smn.SafetyMonitorNode()
    node.safety_rules['safety_zones'] = [
        {
            'name': 'robot_workspace',
            'type': 'cylinder',
            'center': [0.0, 0.0, 0.0],
            'radius': 1.0,
            'height': 2.0,
            'restricted_objects': ['human'],
        }
    ]
    node.environment_config = {
        'objects': [
            {
                'id': 'h1',
                'type': 'human',
                'position': [0.0, 0.0, 1.0],
                'dimensions': [0.2, 0.2, 1.8],
            }
        ]
    }
    logger = node.get_logger()
    node.check_safety_zones()
    logger.warning.assert_called_once()
