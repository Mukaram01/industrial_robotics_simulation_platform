import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "src"))


def _import_node(monkeypatch, plan_result):
    _setup_ros_stubs(monkeypatch)

    # std_msgs Header
    import std_msgs
    class Header:
        pass
    monkeypatch.setattr(std_msgs.msg, "Header", Header, raising=False)

    # geometry messages
    geom = types.ModuleType("geometry_msgs")
    geom.msg = types.ModuleType("geometry_msgs.msg")

    class Pose:
        def __init__(self):
            pass

    class PoseStamped:
        pass

    class Point:
        pass

    class Quaternion:
        pass

    geom.msg.Pose = Pose
    geom.msg.PoseStamped = PoseStamped
    geom.msg.Point = Point
    geom.msg.Quaternion = Quaternion
    monkeypatch.setitem(sys.modules, "geometry_msgs", geom)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geom.msg)

    # tf2 stubs
    tf2_ros = types.ModuleType("tf2_ros")
    class Buffer:
        pass
    class TransformListener:
        def __init__(self, buffer, node):
            pass
    tf2_ros.Buffer = Buffer
    tf2_ros.TransformListener = TransformListener
    monkeypatch.setitem(sys.modules, "tf2_ros", tf2_ros)

    tf2_geom = types.ModuleType("tf2_geometry_msgs")
    tf2_geom.do_transform_pose = lambda pose, transform: pose
    monkeypatch.setitem(sys.modules, "tf2_geometry_msgs", tf2_geom)

    # apm messages
    apm = types.ModuleType("apm_msgs")
    apm.msg = types.ModuleType("apm_msgs.msg")
    apm.msg.DetectedObject = object
    apm.msg.DetectedObjectArray = object
    monkeypatch.setitem(sys.modules, "apm_msgs", apm)
    monkeypatch.setitem(sys.modules, "apm_msgs.msg", apm.msg)

    # moveit messages
    moveit_msgs = types.ModuleType("moveit_msgs")
    moveit_msgs.msg = types.ModuleType("moveit_msgs.msg")
    moveit_msgs.srv = types.ModuleType("moveit_msgs.srv")
    moveit_msgs.msg.MoveItErrorCodes = object
    moveit_msgs.msg.RobotTrajectory = object
    moveit_msgs.srv.GetPositionIK = object
    monkeypatch.setitem(sys.modules, "moveit_msgs", moveit_msgs)
    monkeypatch.setitem(sys.modules, "moveit_msgs.msg", moveit_msgs.msg)
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", moveit_msgs.srv)

    # moveit commander
    mc = types.ModuleType("moveit_commander")
    mc.roscpp_initialize = lambda *a, **k: None
    mc.RobotCommander = object
    mc.PlanningSceneInterface = object
    group = MagicMock()
    group.plan.return_value = plan_result
    group.set_pose_target = MagicMock()
    group.execute = MagicMock()
    group.clear_pose_targets = MagicMock()
    mc.MoveGroupCommander = MagicMock(return_value=group)
    monkeypatch.setitem(sys.modules, "moveit_commander", mc)

    sys.modules.pop("fmm_core.fmm_core.pick_and_place_node", None)
    from fmm_core.fmm_core import pick_and_place_node as ppn

    return ppn, group


def test_move_to_pose_success(monkeypatch):
    plan = object()
    ppn, group = _import_node(monkeypatch, (True, plan, 0.0, object()))
    node = ppn.PickAndPlaceNode.__new__(ppn.PickAndPlaceNode)
    node.move_group = group
    node.get_logger = lambda: MagicMock(error=MagicMock())

    result = node.move_to_pose(ppn.Pose())

    assert result is True
    assert group.set_pose_target.call_count == 1
    group.execute.assert_called_once_with(plan, wait=True)
    group.clear_pose_targets.assert_called_once()


def test_move_to_pose_failure(monkeypatch):
    ppn, group = _import_node(monkeypatch, (False, None, 0.0, object()))
    node = ppn.PickAndPlaceNode.__new__(ppn.PickAndPlaceNode)
    node.move_group = group
    node.get_logger = lambda: MagicMock(error=MagicMock())

    result = node.move_to_pose(ppn.Pose())

    assert result is False
    group.execute.assert_not_called()
    group.clear_pose_targets.assert_called_once()
