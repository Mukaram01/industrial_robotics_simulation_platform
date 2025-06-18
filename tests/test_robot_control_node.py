import sys
from pathlib import Path
from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))


def _msg(data: str):
    from std_msgs.msg import String
    m = String()
    m.data = data
    return m


def test_command_processing(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('simulation_core.robot_control_node', None)
    sys.modules.pop('simulation_core', None)
    from simulation_core import robot_control_node as rcn

    node = rcn.RobotControlNode()

    node.jog_pub.publish.reset_mock()
    node.command_callback(_msg('jog X Y'))
    node.jog_pub.publish.assert_called_once()
    out = node.jog_pub.publish.call_args[0][0]
    assert out.data == 'X Y'
    assert node.jog_pub._topic == '/robot/jog'

    node.waypoint_pub.publish.reset_mock()
    node.command_callback(_msg('record_waypoint'))
    node.waypoint_pub.publish.assert_called_once()
    out = node.waypoint_pub.publish.call_args[0][0]
    assert out.data == 'record_waypoint'
    assert node.waypoint_pub._topic == '/robot/waypoint'

    node.waypoint_pub.publish.reset_mock()
    node.command_callback(_msg('clear_waypoints'))
    node.waypoint_pub.publish.assert_called_once()
    out = node.waypoint_pub.publish.call_args[0][0]
    assert out.data == 'clear_waypoints'

    node.sequence_pub.publish.reset_mock()
    node.command_callback(_msg('execute_sequence'))
    node.sequence_pub.publish.assert_called_once()
    out = node.sequence_pub.publish.call_args[0][0]
    assert out.data == 'execute'
    assert node.sequence_pub._topic == '/robot/execute_sequence'

    node.jog_pub.publish.reset_mock()
    node.waypoint_pub.publish.reset_mock()
    node.sequence_pub.publish.reset_mock()

    for bad in ['', 'jog 1', 'unknown']:
        node.command_callback(_msg(bad))

    assert not node.jog_pub.publish.called
    assert not node.waypoint_pub.publish.called
    assert not node.sequence_pub.publish.called
