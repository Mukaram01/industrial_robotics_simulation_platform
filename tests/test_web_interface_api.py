import sys
import types
from pathlib import Path
from unittest.mock import MagicMock
import io
import json
import yaml


# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))


def _setup_ros_stubs(monkeypatch):
    """Create minimal ROS stubs so WebInterfaceNode can be imported."""
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class DummyNode:
        def __init__(self, *args, **kwargs):
            self.params = {}

        def declare_parameters(self, ns, params):
            for name, value in params:
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

            class Logger:
                def info(self, *a, **k):
                    pass

                def error(self, *a, **k):
                    pass

                def warning(self, *a, **k):
                    pass
            return Logger()

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    rclpy_stub.init = lambda *a, **kw: None
    rclpy_stub.shutdown = lambda *a, **kw: None

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

    sensor_stub = types.ModuleType('sensor_msgs')
    sensor_stub.msg = types.ModuleType('sensor_msgs.msg')
    sensor_stub.msg.Image = Msg
    class JS:
        def __init__(self):
            self.name = []
            self.position = []
            self.velocity = []
    sensor_stub.msg.JointState = JS
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_stub)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_stub.msg)

    cv_bridge_stub = types.ModuleType('cv_bridge')

    class CvBridge:
        def imgmsg_to_cv2(self, *a, **k):
            return None
    cv_bridge_stub.CvBridge = CvBridge
    monkeypatch.setitem(sys.modules, 'cv_bridge', cv_bridge_stub)

    cv2_stub = types.ModuleType('cv2')
    cv2_stub.imencode = lambda *a, **k: (True, b'')
    monkeypatch.setitem(sys.modules, 'cv2', cv2_stub)

    import flask
    monkeypatch.setitem(sys.modules, 'flask', flask)

    ament_stub = types.ModuleType('ament_index_python')
    ament_stub.packages = types.ModuleType('ament_index_python.packages')

    def get_pkg_share(_):
        return str(ROOT / 'src' / 'web_interface_frontend')
    ament_stub.packages.get_package_share_directory = get_pkg_share
    monkeypatch.setitem(sys.modules, 'ament_index_python', ament_stub)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', ament_stub.packages)


def test_place_api_publishes_and_logs(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/place', json={'location': 'bin_red'})
    assert res.status_code == 200

    assert node.command_pub.publish.called
    msg = node.command_pub.publish.call_args[0][0]
    assert msg.data == 'place bin_red'
    assert node.command_pub._topic == '/simulation/command'

    logger_mock.log.assert_called_once_with('place', {'location': 'bin_red'})


def test_upload_scenario_duplicate_rejected(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    node.config_dir = str(tmp_path)

    # create an existing scenario file
    existing = tmp_path / 'foo.yaml'
    existing.write_text('name: foo')

    client = node.app.test_client()
    data = {
        'scenario_id': 'foo',
        'file': (io.BytesIO(b'name: foo2'), 'foo.yaml'),
    }
    res = client.post('/api/scenarios', data=data, content_type='multipart/form-data')

    assert res.status_code == 409


def test_upload_scenario_invalid_extension(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    node.config_dir = str(tmp_path)

    client = node.app.test_client()
    data = {
        'scenario_id': 'bar',
        'file': (io.BytesIO(b'name: bar'), 'bar.txt'),
    }
    res = client.post('/api/scenarios', data=data, content_type='multipart/form-data')

    assert res.status_code == 400
    assert not (tmp_path / 'bar.yaml').exists()


def test_upload_scenario_invalid_yaml(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    node.config_dir = str(tmp_path)

    client = node.app.test_client()
    data = {
        'scenario_id': 'baz',
        'file': (io.BytesIO(b': invalid'), 'baz.yaml'),
    }
    res = client.post('/api/scenarios', data=data, content_type='multipart/form-data')

    assert res.status_code == 400
    assert not (tmp_path / 'baz.yaml').exists()


def test_pick_api_publishes_and_logs(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/pick', json={'object_id': '42'})
    assert res.status_code == 200

    assert node.command_pub.publish.called
    msg = node.command_pub.publish.call_args[0][0]
    assert msg.data == 'pick 42'
    assert node.command_pub._topic == '/simulation/command'

    logger_mock.log.assert_called_once_with('pick', {'object_id': '42'})


def test_objects_api_returns_latest(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    node.latest_objects = [{'id': 1, 'class': 'red'}]
    client = node.app.test_client()

    res = client.get('/api/objects')
    assert res.status_code == 200
    assert res.json['objects'][0]['id'] == 1


def test_joint_state_emits_socket_event(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    emit_mock = MagicMock()
    node.socketio.emit = emit_mock

    js_msg = win.JointState()
    js_msg.name = ['j1']
    js_msg.position = [1.0]
    js_msg.velocity = [0.5]

    node.joint_state_callback(js_msg)

    emit_mock.assert_called_with('joint_state_update', {
        'name': ['j1'],
        'position': [1.0],
        'velocity': [0.5],
    })

def test_jog_api_publishes(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/jog', json={'joint': 1, 'delta': 0.1})
    assert res.status_code == 200
    msg = node.command_pub.publish.call_args[0][0]
    assert msg.data == 'jog 1 0.1'
    logger_mock.log.assert_called_once_with('jog', {'joint': 1, 'delta': 0.1})

def test_waypoint_api_execute(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/waypoint', json={'action': 'execute'})
    assert res.status_code == 200
    msg = node.command_pub.publish.call_args[0][0]
    assert msg.data == 'execute_sequence'
    logger_mock.log.assert_called_once_with('waypoint', {'action': 'execute'})

def test_load_scenario_endpoint(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/scenarios/foo/load')
    assert res.status_code == 200
    assert node.config_pub.publish.called
    msg = node.config_pub.publish.call_args[0][0]
    assert json.loads(msg.data) == {'scenario': 'foo'}
    logger_mock.log.assert_called_once_with('load_scenario', {'id': 'foo'})

def test_save_scenario_endpoint(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    logger_mock = MagicMock()
    monkeypatch.setattr(win, 'ActionLogger', MagicMock(return_value=logger_mock))
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    node.config_dir = str(tmp_path)
    client = node.app.test_client()

    res = client.put('/api/scenarios/test', json={'config': {'a': 1}})
    assert res.status_code == 200

    path = tmp_path / 'test.yaml'
    assert path.exists()
    assert yaml.safe_load(path.read_text())['a'] == 1

    assert node.config_pub.publish.called
    msg = node.config_pub.publish.call_args[0][0]
    data = json.loads(msg.data)
    assert data['update_scenario']['name'] == 'test'
    logger_mock.log.assert_called_with('save_scenario', {'id': 'test'})

