import sys
import types
from pathlib import Path
import pytest

try:
    import numpy as np
except ModuleNotFoundError:
    pytest.skip("NumPy is required for pose estimation tests", allow_module_level=True)

try:
    import yaml  # noqa: F401
except ModuleNotFoundError:
    pytest.skip("PyYAML is required for pose estimation tests", allow_module_level=True)
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src' / 'advanced_perception'))


def _setup_stubs(monkeypatch):
    rclpy_stub = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    exec_mod = types.ModuleType('rclpy.executors')

    class Logger:
        def __init__(self):
            self.info = MagicMock()
            self.warn = MagicMock()
            self.warning = MagicMock()
            self.error = MagicMock()
            self.debug = MagicMock()

    class DummyNode:
        def __init__(self, *a, **k):
            self.params = {}
            self._logger = Logger()

        def declare_parameter(self, name, value):
            self.params[name] = value

        class P:
            def __init__(self, value):
                self.value = value

        def get_parameter(self, name):
            return self.P(self.params.get(name))

        def create_subscription(self, *a, **k):
            return MagicMock()

        def create_publisher(self, *a, **k):
            return MagicMock()

        def get_logger(self):
            return self._logger

    node_mod.Node = DummyNode
    rclpy_stub.node = node_mod
    exec_mod.MultiThreadedExecutor = MagicMock()
    rclpy_stub.executors = exec_mod
    rclpy_stub.init = lambda *a, **k: None
    rclpy_stub.shutdown = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_stub)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_mod)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', exec_mod)

    sensor_stub = types.ModuleType('sensor_msgs')
    sensor_stub.msg = types.ModuleType('sensor_msgs.msg')

    class Image:
        pass

    class CameraInfo:
        def __init__(self):
            self.k = [0.0] * 9
            self.d = []

    sensor_stub.msg.Image = Image
    sensor_stub.msg.CameraInfo = CameraInfo
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_stub)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_stub.msg)

    geometry_stub = types.ModuleType('geometry_msgs')
    geometry_stub.msg = types.ModuleType('geometry_msgs.msg')
    geometry_stub.msg.PoseStamped = object
    monkeypatch.setitem(sys.modules, 'geometry_msgs', geometry_stub)
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', geometry_stub.msg)

    cv_bridge_stub = types.ModuleType('cv_bridge')

    class DummyBridge:
        def imgmsg_to_cv2(self, *a, **k):
            return np.zeros((10, 10), dtype=np.uint8)
    cv_bridge_stub.CvBridge = DummyBridge
    monkeypatch.setitem(sys.modules, 'cv_bridge', cv_bridge_stub)

    cv2_stub = types.ModuleType('cv2')
    cv2_stub.cvtColor = lambda *a, **k: np.zeros((10, 10, 3), dtype=np.uint8)
    cv2_stub.inRange = lambda *a, **k: np.zeros((10, 10), dtype=np.uint8)
    cv2_stub.morphologyEx = lambda *a, **k: np.zeros((10, 10), dtype=np.uint8)
    cv2_stub.findContours = lambda *a, **k: ([], None)
    cv2_stub.drawContours = lambda *a, **k: None
    cv2_stub.rectangle = lambda *a, **k: None
    cv2_stub.circle = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, 'cv2', cv2_stub)

    apm_stub = types.ModuleType('apm_msgs')
    apm_stub.msg = types.ModuleType('apm_msgs.msg')

    class Header:
        pass

    class Pose:
        class Position:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        class Orientation:
            def __init__(self):
                self.w = 1.0
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        def __init__(self):
            self.position = Pose.Position()
            self.orientation = Pose.Orientation()

    class Dimensions:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class DetectedObject:
        def __init__(self):
            self.header = Header()
            self.id = 0
            self.class_id = 0
            self.class_name = ''
            self.confidence = 0.0
            self.pose = Pose()
            self.dimensions = Dimensions()

    class DetectedObjectArray:
        def __init__(self):
            self.header = Header()
            self.objects = []

    class Detection2D:
        def __init__(self):
            self.class_id = 0
            self.class_name = ''
            self.score = 0.0
            self.bbox = types.SimpleNamespace(
                x_offset=0, y_offset=0, width=0, height=0, do_rectify=False
            )

    class Detection2DArray:
        def __init__(self):
            self.header = Header()
            self.detections = []

    apm_stub.msg.DetectedObject = DetectedObject
    apm_stub.msg.DetectedObjectArray = DetectedObjectArray
    apm_stub.msg.Detection2D = Detection2D
    apm_stub.msg.Detection2DArray = Detection2DArray
    monkeypatch.setitem(sys.modules, 'apm_msgs', apm_stub)
    monkeypatch.setitem(sys.modules, 'apm_msgs.msg', apm_stub.msg)


def test_process_without_camera_info(monkeypatch):
    _setup_stubs(monkeypatch)
    sys.modules.pop('advanced_perception.pose_estimation_node', None)
    from advanced_perception.pose_estimation_node import PoseEstimationNode
    import apm_msgs.msg as msg

    node = PoseEstimationNode()
    node.camera_matrix = None
    node.latest_rgb_image = np.zeros((10, 10, 3), dtype=np.uint8)
    node.latest_depth_image = np.ones((10, 10), dtype=np.uint16)

    det = msg.Detection2D()
    det.bbox.x_offset = 1
    det.bbox.y_offset = 1
    det.bbox.width = 2
    det.bbox.height = 2

    det_array = msg.Detection2DArray()
    det_array.detections = [det]
    det_array.header = object()
    node.latest_segmented_objects = det_array

    # Should not raise even though camera info is missing
    node.process_data()
