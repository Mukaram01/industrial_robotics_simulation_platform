import sqlite3
import json
import pytest

try:
    import yaml  # noqa: F401
except ModuleNotFoundError:
    pytest.skip("PyYAML is required for core logic tests", allow_module_level=True)

# Ensure packages under src/ are importable
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))

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

from web_interface_backend.action_logger import ActionLogger  # noqa: E402
from simulation_core import environment_configurator_node as ec  # noqa: E402

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
    def warning(self, *args, **kwargs):
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


def test_action_logger_handles_db_error(tmp_path):
    db_path = tmp_path / 'actions.db'
    logger = ActionLogger(str(db_path))
    logger._conn.close()  # simulate broken connection without updating attribute
    logger.log('bad')  # should not raise
    conn = sqlite3.connect(db_path)
    count = conn.execute('SELECT COUNT(*) FROM actions').fetchone()[0]
    assert count == 0


def test_action_logger_get_recent(tmp_path):
    db_path = tmp_path / 'actions.db'
    logger = ActionLogger(str(db_path))
    for i in range(3):
        logger.log(f'action{i}', {'id': i})

    recent = logger.get_recent_actions(limit=2)
    assert len(recent) == 2
    assert recent[0]['action'] == 'action2'
    assert recent[1]['action'] == 'action1'


def test_action_logger_creates_dir(tmp_path):
    db_path = tmp_path / 'logs' / 'actions' / 'actions.db'
    logger = ActionLogger(str(db_path))
    assert db_path.parent.exists()
    logger.log('test')
    logger.close()
    assert db_path.exists()


def test_scenario_file_cycle(tmp_path):
    dummy = make_dummy(tmp_path)

    data = {'name': 'test', 'description': 'desc', 'config': {'foo': 'bar'}}
    ec.EnvironmentConfiguratorNode.save_scenario(dummy, data)
    yaml_path = tmp_path / 'test.yaml'
    assert yaml_path.exists()

    dummy.environment_config = {}
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'test')
    assert dummy.environment_config['foo'] == 'bar'

    ec.EnvironmentConfiguratorNode.delete_scenario(dummy, 'test')
    assert not yaml_path.exists()

    # JSON scenario
    json_path = tmp_path / 'test.json'
    json_path.write_text(json.dumps({'foo': 'baz'}))

    dummy.environment_config = {}
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'test')
    assert dummy.environment_config['foo'] == 'baz'


def test_save_scenario_creates_dir(tmp_path):
    config_dir = tmp_path / 'configs' / 'scenarios'
    dummy = make_dummy(config_dir)

    data = {'name': 'auto', 'config': {}}
    ec.EnvironmentConfiguratorNode.save_scenario(dummy, data)

    assert config_dir.exists()
    assert (config_dir / 'auto.yaml').exists()


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


def test_empty_config_results_in_empty_config(tmp_path):
    dummy = make_dummy(tmp_path)

    # Empty YAML
    (tmp_path / 'empty.yaml').write_text('')
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'empty')
    assert dummy.environment_config == {}

    # Empty JSON
    (tmp_path / 'empty.json').write_text('')
    ec.EnvironmentConfiguratorNode.load_scenario(dummy, 'empty')
    assert dummy.environment_config == {}


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


def test_web_interface_logger_initialization_order(tmp_path, monkeypatch):
    """Ensure data directory exists before ActionLogger is created."""
    import types
    import importlib
    import threading

    data_dir = tmp_path / "data"

    class DummyLogger:
        def info(self, *a, **k):
            pass
        def error(self, *a, **k):
            pass
        def warn(self, *a, **k):
            pass
        def warning(self, *a, **k):
            pass

    class DummyNode:
        def __init__(self, name):
            self.params = {
                'port': 8080,
                'host': '0.0.0.0',
                'config_dir': '',
                'data_dir': str(data_dir),
                'save_images': False,
                'allow_unsafe_werkzeug': True,
                'log_db_path': '',
                'jpeg_quality': 75,
                'detected_objects_topic': '/apm/detection/objects',
                'joint_states_topic': '/joint_states',
                'auto_open_browser': False,
                'secret_key': 'dummy_key',
            }

        def declare_parameters(self, ns, params):
            pass

        class Param:
            def __init__(self, val):
                self.value = val

        def get_parameter(self, name):
            return self.Param(self.params[name])

        def create_publisher(self, *a, **kw):
            pass

        def create_subscription(self, *a, **kw):
            pass

        def get_logger(self):
            return DummyLogger()

    stub_modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.ModuleType('std_msgs.msg'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'cv_bridge': types.ModuleType('cv_bridge'),
        'flask': __import__('flask'),
        'flask_socketio': types.ModuleType('flask_socketio'),
        'ament_index_python': types.ModuleType('ament_index_python'),
        'ament_index_python.packages': types.ModuleType('ament_index_python.packages'),
        'cv2': types.ModuleType('cv2'),
        'numpy': types.ModuleType('numpy'),
        'yaml': types.ModuleType('yaml'),
    }
    stub_modules['numpy'].typing = types.SimpleNamespace(NDArray=object)

    stub_modules['rclpy'].node = stub_modules['rclpy.node']
    stub_modules['rclpy.node'].Node = DummyNode

    stub_modules['std_msgs'].msg = stub_modules['std_msgs.msg']
    stub_modules['std_msgs.msg'].String = object
    stub_modules['std_msgs.msg'].Bool = object

    stub_modules['sensor_msgs'].msg = stub_modules['sensor_msgs.msg']
    stub_modules['sensor_msgs.msg'].Image = object
    class JS:
        pass
    stub_modules['sensor_msgs.msg'].JointState = JS

    class DummyCvBridge:
        pass
    stub_modules['cv_bridge'].CvBridge = DummyCvBridge

    import flask as real_flask

    class DummyFlask(real_flask.Flask):
        def __init__(self, *a, **k):
            super().__init__(__name__)

    stub_modules['flask'].Flask = DummyFlask

    class DummySocketIO:
        def __init__(self, app, cors_allowed_origins=None):
            pass

        def on(self, *a, **k):
            def decorator(f):
                return f

            return decorator

        def emit(self, *a, **k):
            pass

        def run(self, *a, **k):
            pass

    stub_modules['flask_socketio'].SocketIO = DummySocketIO

    stub_modules['ament_index_python'].packages = stub_modules['ament_index_python.packages']
    stub_modules['ament_index_python.packages'].get_package_share_directory = lambda pkg: str(data_dir)

    stub_modules['yaml'].safe_load = lambda *a, **k: {}

    for name, mod in stub_modules.items():
        sys.modules[name] = mod

    monkeypatch.setenv("WEB_INTERFACE_SECRET", "dummy_key")

    win = importlib.import_module('web_interface_backend.web_interface_node')

    class DummyThread:
        def __init__(self, target):
            self.target = target

        def start(self):
            pass

    threading.Thread = DummyThread

    node = win.WebInterfaceNode()

    try:
        assert data_dir.exists()
        expected = str(data_dir / 'actions.db')
        assert node.log_db_path == expected
        assert node.action_logger.db_path == expected
    finally:
        for name in stub_modules.keys():
            sys.modules.pop(name, None)
        sys.modules.pop('web_interface_backend.web_interface_node', None)
        pkg = sys.modules.get('web_interface_backend')
        if pkg and hasattr(pkg, 'web_interface_node'):
            delattr(pkg, 'web_interface_node')
