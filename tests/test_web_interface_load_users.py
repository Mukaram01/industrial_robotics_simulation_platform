import sys
from pathlib import Path
from unittest.mock import MagicMock

from test_utils import _setup_ros_stubs

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))


def test_load_users_valid_file(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)

    sys.modules.pop('web_interface_backend.web_interface_node', None)
    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())
    monkeypatch.setattr(win.WebInterfaceNode, 'run_server', lambda self: None)

    users_file = tmp_path / 'users.yaml'
    users_file.write_text('alice: secret\nbob: pass')

    node = win.WebInterfaceNode()
    node.config_dir = str(tmp_path)
    users = node._load_users()

    assert users == {'alice': 'secret', 'bob': 'pass'}
