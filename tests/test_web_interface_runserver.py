import sys
import types
from unittest.mock import MagicMock

# reuse helper from API tests
from test_web_interface_api import _setup_ros_stubs


def test_run_server_fallback(monkeypatch):
    _setup_ros_stubs(monkeypatch)

    class DummySocketIO:
        def __init__(self, app, cors_allowed_origins=None):
            self.calls = []

        def on(self, *a, **k):
            def decorator(f):
                return f
            return decorator

        def emit(self, *a, **k):
            pass

        def run(self, *a, **kw):
            self.calls.append(kw)
            if len(self.calls) == 1:
                raise TypeError('unexpected keyword')

        def stop(self):
            pass

    fsio_mod = types.ModuleType('flask_socketio')
    fsio_mod.SocketIO = DummySocketIO
    monkeypatch.setitem(sys.modules, 'flask_socketio', fsio_mod)

    import threading

    class DummyThread:
        def __init__(self, target, *a, **k):
            self.target = target
            self.daemon = True

        def start(self):
            pass

    monkeypatch.setattr(threading, 'Thread', DummyThread)

    sys.modules.pop('web_interface_backend.web_interface_node', None)
    from web_interface_backend import web_interface_node as win
    import flask
    win.Flask = flask.Flask
    monkeypatch.setattr(win, 'SocketIO', DummySocketIO)

    monkeypatch.setattr(win, 'ActionLogger', MagicMock())

    node = win.WebInterfaceNode()
    logger = MagicMock()
    node.get_logger = lambda: logger

    node.run_server()

    assert len(node.socketio.calls) == 2
    assert 'allow_unsafe_werkzeug' in node.socketio.calls[0]
    assert 'allow_unsafe_werkzeug' not in node.socketio.calls[1]
    logger.warning.assert_called_once()
