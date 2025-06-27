import sys
from pathlib import Path
from unittest.mock import MagicMock
import io
import json
import pytest

try:
    import yaml
except ModuleNotFoundError:
    pytest.skip("PyYAML is required for web interface tests", allow_module_level=True)

from test_utils import _setup_ros_stubs


# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))


def _login(client):
    return client.post('/login', data={'username': 'admin', 'password': 'admin'})


def test_login_required_redirect(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)

    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    node = win.WebInterfaceNode()
    client = node.app.test_client()

    res = client.post('/api/place', json={'location': 'a'})
    assert res.status_code in (302, 401)

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
    _login(client)

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
    _login(client)
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
    _login(client)
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
    _login(client)
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
    _login(client)

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
    _login(client)

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
    _login(client)

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
    _login(client)

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
    _login(client)

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
    _login(client)

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
