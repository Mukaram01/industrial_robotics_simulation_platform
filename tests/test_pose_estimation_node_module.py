import sys
import types
from pathlib import Path

import pytest
from test_utils import _setup_ros_stubs

np = pytest.importorskip('numpy')

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src' / 'advanced_perception'))


def _setup_apm(monkeypatch):
    apm = types.ModuleType('apm_msgs')
    apm.msg = types.ModuleType('apm_msgs.msg')

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
            self.header = None
            self.detections = []

    class Pose:
        def __init__(self):
            self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation = types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)

    class Dimensions:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class DetectedObject:
        def __init__(self):
            self.header = None
            self.id = 0
            self.class_id = 0
            self.class_name = ''
            self.confidence = 0.0
            self.pose = Pose()
            self.dimensions = Dimensions()

    class DetectedObjectArray:
        def __init__(self):
            self.header = None
            self.objects = []

    apm.msg.Detection2D = Detection2D
    apm.msg.Detection2DArray = Detection2DArray
    apm.msg.DetectedObject = DetectedObject
    apm.msg.DetectedObjectArray = DetectedObjectArray
    monkeypatch.setitem(sys.modules, 'apm_msgs', apm)
    monkeypatch.setitem(sys.modules, 'apm_msgs.msg', apm.msg)


class DummyBridge:
    def imgmsg_to_cv2(self, *a, **k):
        return np.zeros((4, 4), dtype=np.uint16)


def _setup_bridge(monkeypatch):
    import cv_bridge

    monkeypatch.setattr(cv_bridge, 'CvBridge', DummyBridge, raising=False)


def test_load_config_defaults(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.pose_estimation_node', None)
    from advanced_perception import pose_estimation_node as pn

    node = pn.PoseEstimationNode()
    assert node.min_depth == 0.1
    assert node.max_depth == 5.0
    assert node.depth_scale == 0.001


def test_load_config_file(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.pose_estimation_node', None)
    from advanced_perception import pose_estimation_node as pn

    cfg = tmp_path / 'pose.yaml'
    cfg.write_text('min_depth: 0.5\nmax_depth: 2.0\ndepth_scale: 0.002\n')

    node = pn.PoseEstimationNode()
    node.pose_estimation_config_path = str(cfg)
    node.load_config()

    assert node.min_depth == 0.5
    assert node.max_depth == 2.0
    assert node.depth_scale == 0.002


def test_process_data_publishes(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.pose_estimation_node', None)
    from advanced_perception import pose_estimation_node as pn
    import apm_msgs.msg as msg

    node = pn.PoseEstimationNode()
    node.camera_matrix = np.eye(3)
    node.latest_rgb_image = np.zeros((4, 4, 3), dtype=np.uint8)
    node.latest_depth_image = np.ones((4, 4), dtype=np.uint16) * 1000

    det = msg.Detection2D()
    det.bbox.width = 2
    det.bbox.height = 2
    det_array = msg.Detection2DArray()
    det_array.detections = [det]
    det_array.header = object()
    node.latest_segmented_objects = det_array

    node.objects_pub.publish.reset_mock()
    node.process_data()

    assert node.objects_pub.publish.called
    out = node.objects_pub.publish.call_args[0][0]
    assert len(out.objects) == 1
