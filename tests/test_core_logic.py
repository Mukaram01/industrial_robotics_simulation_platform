import os
import sqlite3
import json

import pytest

# Ensure packages under src/ are importable
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))

# Provide stub ROS modules so we can import ROS-dependent modules without ROS
import types

# Create temporary stub modules so ROS-dependent imports succeed
rclpy_stub = types.ModuleType('rclpy')
rclpy_stub.node = types.ModuleType('rclpy.node')
rclpy_stub.node.Node = object
std_msgs_stub = types.ModuleType('std_msgs')
std_msgs_stub.msg = types.ModuleType('std_msgs.msg')
class _DummyMsg:
    pass
std_msgs_stub.msg.String = _DummyMsg
std_msgs_stub.msg.Bool = _DummyMsg
sys.modules['rclpy'] = rclpy_stub
sys.modules['rclpy.node'] = rclpy_stub.node
sys.modules['std_msgs'] = std_msgs_stub
sys.modules['std_msgs.msg'] = std_msgs_stub.msg

from simulation_tools.simulation_tools.action_logger import ActionLogger
from simulation_tools.simulation_tools import environment_configurator_node as ec

# Remove stub modules so other tests that expect missing ROS will skip
for mod in ['rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg']:
    sys.modules.pop(mod, None)

class DummyLogger:
    def info(self, *args, **kwargs):
        pass
    def error(self, *args, **kwargs):
        pass
    def warn(self, *args, **kwargs):
        pass


class DummyEC(ec.EnvironmentConfiguratorNode):
    def __init__(self, tmpdir):
        # Skip ROS Node initialization
        self.config_dir = str(tmpdir)
        self.environment_config = {}
        self.current_scenario = ''
        self.physics_enabled = True
        self.record_metrics = False
        self.error_simulation_rate = 0.0
        self.running = False
        self.get_logger = lambda: DummyLogger()


def make_dummy(tmpdir):
    return DummyEC(tmpdir)


def test_action_logger_basic(tmp_path):
    db_path = tmp_path / 'actions.db'
    logger = ActionLogger(str(db_path))
    logger.log('pick', {'item': 1})
    conn = sqlite3.connect(db_path)
    row = conn.execute('SELECT action, details FROM actions').fetchone()
    assert row[0] == 'pick'
    assert json.loads(row[1]) == {'item': 1}


def test_action_logger_close(tmp_path):
    db_path = tmp_path / 'actions.db'
    with ActionLogger(str(db_path)) as logger:
        logger.log('foo')
    assert logger._conn is None
    conn = sqlite3.connect(db_path)
    count = conn.execute('SELECT COUNT(*) FROM actions').fetchone()[0]
    assert count == 1


def test_scenario_file_cycle(tmp_path):
    dummy = make_dummy(tmp_path)

    data = {'name': 'test', 'description': 'desc', 'config': {'foo': 'bar'}}
    ec.EnvironmentConfiguratorNode.save_scenario(dummy, data)
    path = tmp_path / 'test.yaml'
    assert path.exists()

    dummy.environment_config = {}
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'test')
    assert dummy.environment_config['foo'] == 'bar'

    ec.EnvironmentConfiguratorNode.delete_scenario(dummy, 'test')
    assert not path.exists()


def test_update_settings(tmp_path):
    dummy = make_dummy(tmp_path)
    ec.EnvironmentConfiguratorNode.update_settings(
        dummy,
        {'simulation': {
            'physics_enabled': False,
            'record_metrics': True,
            'error_simulation_rate': 0.2,
        }}
    )
    assert dummy.physics_enabled is False
    assert dummy.record_metrics is True
    assert dummy.error_simulation_rate == 0.2


def test_load_missing_uses_default(tmp_path):
    dummy = make_dummy(tmp_path)
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'missing')
    default = ec.EnvironmentConfiguratorNode.get_default_config(dummy)
    assert dummy.environment_config['description'] == default['description']


def test_error_sim_rate_bounds(tmp_path):
    dummy = make_dummy(tmp_path)
    ec.EnvironmentConfiguratorNode.update_settings(
        dummy,
        {'simulation': {'error_simulation_rate': 1.5}}
    )
    assert dummy.error_simulation_rate == 1.0

    ec.EnvironmentConfiguratorNode.update_settings(
        dummy,
        {'simulation': {'error_simulation_rate': -0.5}}
    )
    assert dummy.error_simulation_rate == 0.0
