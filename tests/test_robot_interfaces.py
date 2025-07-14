import importlib
import sys
from pathlib import Path
from test_utils import _setup_ros_stubs

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'robot_interfaces'))


def test_delta_interface_docstring(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('robot_interfaces.delta_interface', None)
    mod = importlib.import_module('robot_interfaces.delta_interface')
    assert mod.__doc__ == 'Delta robot specific interface helpers.'


def test_ur5_interface_docstring(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('robot_interfaces.ur5_interface', None)
    mod = importlib.import_module('robot_interfaces.ur5_interface')
    assert mod.__doc__ == 'UR5 robot interface helpers.'


def _msg(data: str):
    from std_msgs.msg import String

    m = String()
    m.data = data
    return m


def test_delta_interface_send_command(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('robot_interfaces.delta_interface', None)
    from robot_interfaces.delta_interface import DeltaInterface
    from rclpy.node import Node

    node = Node()
    iface = DeltaInterface(node)

    iface.command_pub.publish.reset_mock()
    iface.send_command('start')
    iface.command_pub.publish.assert_called_once()
    out = iface.command_pub.publish.call_args[0][0]
    assert out.data == 'start'

    iface._status_callback(_msg('running'))
    assert iface.get_status() == 'running'

    from sensor_msgs.msg import JointState

    js = JointState()
    js.name = ['j1']
    js.position = [1.0]
    assert DeltaInterface.convert_joint_state(js) == {'j1': 1.0}


def test_ur5_interface_send_command(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('robot_interfaces.ur5_interface', None)
    from robot_interfaces.ur5_interface import UR5Interface
    from rclpy.node import Node

    node = Node()
    iface = UR5Interface(node)

    iface.command_pub.publish.reset_mock()
    iface.send_command('stop')
    iface.command_pub.publish.assert_called_once()
    out = iface.command_pub.publish.call_args[0][0]
    assert out.data == 'stop'

    iface._status_callback(_msg('idle'))
    assert iface.get_status() == 'idle'
