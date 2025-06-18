import sys
import types
from unittest.mock import MagicMock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))


def _setup_ros_stubs(monkeypatch, param_overrides=None):
    """Create ROS stub modules for testing.

    Parameters
    ----------
    monkeypatch : pytest.MonkeyPatch
        Fixture used to inject the stubs.
    param_overrides : dict, optional
        Optional parameter defaults used by :class:`DummyNode`.
    """
    param_overrides = param_overrides or {}
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class DummyLogger:
        def __init__(self):
            self.info = MagicMock()
            self.warn = MagicMock()
            self.warning = MagicMock()
            self.error = MagicMock()

    class DummyClient:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            fut = MagicMock()
            fut.result.return_value = types.SimpleNamespace(
                scene=types.SimpleNamespace(world=types.SimpleNamespace(collision_objects=[]))
            )
            return fut

    class DummyNode:
        def __init__(self, *a, **k):
            self.params = {}
            self._logger = DummyLogger()

        def declare_parameter(self, name, value):
            value = param_overrides.get(name, value)
            self.params[name] = value
            return types.SimpleNamespace(value=value)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.params.get(name))

        def create_publisher(self, *a, **k):
            pub = MagicMock()
            pub._topic = a[1] if len(a) > 1 else k.get('topic')
            return pub

        def create_subscription(self, *a, **k):
            return MagicMock()

        def create_client(self, *a, **k):
            return DummyClient()

        def create_timer(self, *a, **k):
            return MagicMock()

        def get_logger(self):
            return self._logger

        def destroy_node(self):
            pass

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None

    time_mod = types.ModuleType('rclpy.time')
    time_mod.Time = lambda *a, **k: None
    rclpy_stub.time = time_mod

    cb_mod = types.ModuleType('rclpy.callback_groups')

    class ReentrantCallbackGroup:
        pass

    class MutuallyExclusiveCallbackGroup:
        pass

    cb_mod.ReentrantCallbackGroup = ReentrantCallbackGroup
    cb_mod.MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup
    rclpy_stub.callback_groups = cb_mod

    exec_mod = types.ModuleType('rclpy.executors')
    exec_mod.MultiThreadedExecutor = MagicMock()
    rclpy_stub.executors = exec_mod

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.time', time_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.callback_groups', cb_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', exec_mod)

    std_msgs = types.ModuleType('std_msgs')
    std_msgs.msg = types.ModuleType('std_msgs.msg')

    class String:
        def __init__(self):
            self.data = ''

    class Header:
        def __init__(self):
            self.frame_id = ''

    std_msgs.msg.String = String
    std_msgs.msg.Header = Header
    monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs.msg)

    geometry = types.ModuleType('geometry_msgs')
    geometry.msg = types.ModuleType('geometry_msgs.msg')

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class PoseStamped:
        def __init__(self):
            self.header = Header()
            self.pose = Pose()

    geometry.msg.Point = Point
    geometry.msg.Quaternion = Quaternion
    geometry.msg.Pose = Pose
    geometry.msg.PoseStamped = PoseStamped
    monkeypatch.setitem(sys.modules, 'geometry_msgs', geometry)
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', geometry.msg)

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs.msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs.msg.JointState = object
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_msgs)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_msgs.msg)

    apm = types.ModuleType('apm_msgs')
    apm.msg = types.ModuleType('apm_msgs.msg')

    class Header:
        def __init__(self):
            self.frame_id = ''

    class Dimensions:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class DetectedObject:
        def __init__(self, oid=0):
            self.header = Header()
            self.id = oid
            self.pose = Pose()
            self.dimensions = Dimensions()

    class DetectedObjectArray:
        def __init__(self):
            self.objects = []
            self.header = Header()

    apm.msg.DetectedObject = DetectedObject
    apm.msg.DetectedObjectArray = DetectedObjectArray
    monkeypatch.setitem(sys.modules, 'apm_msgs', apm)
    monkeypatch.setitem(sys.modules, 'apm_msgs.msg', apm.msg)

    moveit_msgs = types.ModuleType('moveit_msgs')
    moveit_msgs.msg = types.ModuleType('moveit_msgs.msg')
    moveit_msgs.srv = types.ModuleType('moveit_msgs.srv')
    moveit_msgs.msg.DisplayTrajectory = object
    moveit_msgs.msg.RobotTrajectory = object
    moveit_msgs.msg.MoveItErrorCodes = object
    moveit_msgs.msg.AttachedCollisionObject = object

    class GetPlanningScene:
        class Request:
            pass

    moveit_msgs.srv.GetPositionIK = object
    moveit_msgs.srv.GetPositionFK = object
    moveit_msgs.srv.GetPlanningScene = GetPlanningScene

    class PlanningScene:
        def __init__(self):
            self.is_diff = False
            self.world = types.SimpleNamespace(collision_objects=[])

    class CollisionObject:
        ADD = 1
        REMOVE = 2
        def __init__(self):
            self.header = Header()
            self.id = ''
            self.operation = 0
            self.primitives = []
            self.primitive_poses = []

    class PlanningSceneComponents:
        WORLD_OBJECT_NAMES = 1
        WORLD_OBJECT_GEOMETRY = 2

    moveit_msgs.msg.PlanningScene = PlanningScene
    moveit_msgs.msg.CollisionObject = CollisionObject
    moveit_msgs.msg.PlanningSceneComponents = PlanningSceneComponents
    monkeypatch.setitem(sys.modules, 'moveit_msgs', moveit_msgs)
    monkeypatch.setitem(sys.modules, 'moveit_msgs.msg', moveit_msgs.msg)
    monkeypatch.setitem(sys.modules, 'moveit_msgs.srv', moveit_msgs.srv)

    shape_msgs = types.ModuleType('shape_msgs')
    shape_msgs.msg = types.ModuleType('shape_msgs.msg')

    class SolidPrimitive:
        BOX = 1
        def __init__(self):
            self.type = 0
            self.dimensions = []

    shape_msgs.msg.SolidPrimitive = SolidPrimitive
    monkeypatch.setitem(sys.modules, 'shape_msgs', shape_msgs)
    monkeypatch.setitem(sys.modules, 'shape_msgs.msg', shape_msgs.msg)

    tf2_ros = types.ModuleType('tf2_ros')

    class Buffer:
        def lookup_transform(self, target, source, time):
            return None

    class TransformListener:
        def __init__(self, buffer, node):
            pass

    tf2_ros.Buffer = Buffer
    tf2_ros.TransformListener = TransformListener
    monkeypatch.setitem(sys.modules, 'tf2_ros', tf2_ros)

    tf2_geom = types.ModuleType('tf2_geometry_msgs')
    tf2_geom.do_transform_pose = lambda pose, transform: pose
    monkeypatch.setitem(sys.modules, 'tf2_geometry_msgs', tf2_geom)

    mc = types.ModuleType('moveit_commander')
    mc.roscpp_initialize = lambda *a, **k: None

    class RobotCommander:
        pass

    class PlanningSceneInterface:
        pass

    class MoveGroupCommander:
        """Minimal MoveGroup stub recording configuration calls."""

        last_instance = None

        def __init__(self, group):
            self.group = group
            self.planning_time = None
            self.num_planning_attempts = None
            self.max_velocity_scaling_factor = None
            self.max_acceleration_scaling_factor = None
            self.workspace = None
            MoveGroupCommander.last_instance = self

        def set_planning_time(self, t):
            self.planning_time = t

        def set_num_planning_attempts(self, n):
            self.num_planning_attempts = n

        def set_max_velocity_scaling_factor(self, f):
            self.max_velocity_scaling_factor = f

        def set_max_acceleration_scaling_factor(self, f):
            self.max_acceleration_scaling_factor = f

        def set_workspace(self, *a):
            self.workspace = a

        def set_pose_target(self, pose):
            pass

        def go(self, wait=True):
            return True

        def stop(self):
            pass

        def clear_pose_targets(self):
            pass

        def plan(self):
            return True, object(), 0.0, object()

        def execute(self, plan, wait=True):
            pass

    mc.RobotCommander = RobotCommander
    mc.PlanningSceneInterface = PlanningSceneInterface
    mc.MoveGroupCommander = MoveGroupCommander
    monkeypatch.setitem(sys.modules, 'moveit_commander', mc)


def test_moveit_interface_init(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('fmm_core.fmm_core.fmm_moveit_interface_node', None)
    from fmm_core.fmm_core import fmm_moveit_interface_node as fim
    import apm_msgs.msg as msg

    node = fim.FMMMoveitInterfaceNode()
    assert node.get_parameter('robot_name').value == 'delta_robot'

    node.publish_status('ready')
    assert node.status_pub.publish.called

    node.detected_objects_callback(msg.DetectedObjectArray())


def test_planning_scene_updater_init(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('fmm_core.fmm_core.planning_scene_updater_node', None)
    from fmm_core.fmm_core import planning_scene_updater_node as psu
    import apm_msgs.msg as msg

    node = psu.PlanningSceneUpdaterNode()
    assert node.get_parameter('update_frequency').value == 10.0

    arr = msg.DetectedObjectArray()
    node.detected_objects_callback(arr)


def test_pick_and_place_node_locations(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('fmm_core.fmm_core.pick_and_place_node', None)
    from fmm_core.fmm_core import pick_and_place_node as ppn

    node = ppn.PickAndPlaceNode()
    pose = node.get_predefined_place_location('home')
    assert pose.position.z >= 0.0


def test_pick_and_place_node_parameters(monkeypatch):
    overrides = {
        'max_velocity_scaling_factor': 0.8,
        'workspace_limits': [-1.0, 1.0, -1.0, 1.0, 0.0, 0.7],
    }

    _setup_ros_stubs(monkeypatch, param_overrides=overrides)
    sys.modules.pop('fmm_core.fmm_core.pick_and_place_node', None)
    from fmm_core.fmm_core import pick_and_place_node as ppn

    ppn.PickAndPlaceNode()

    mg = sys.modules['moveit_commander'].MoveGroupCommander.last_instance
    assert mg.planning_time == 5.0
    assert mg.num_planning_attempts == 10
    assert mg.max_velocity_scaling_factor == 0.8
    assert mg.max_acceleration_scaling_factor == 0.5
    assert mg.workspace == (overrides['workspace_limits'],)


def test_sorting_demo_control(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('fmm_core.fmm_core.sorting_demo_node', None)
    from fmm_core.fmm_core import sorting_demo_node as sdn

    class DummyThread:
        def __init__(self, target, *a, **k):
            self.started = False
            self.target = target

        def start(self):
            self.started = True

    monkeypatch.setattr(sdn.threading, 'Thread', DummyThread)

    node = sdn.SortingDemoNode()
    msg = types.SimpleNamespace(data='ready')
    node.status_callback(msg)
    node.demo_control_callback()
    assert node.demo_running is True
    assert isinstance(node.demo_thread, DummyThread)
    assert node.demo_thread.started is True
