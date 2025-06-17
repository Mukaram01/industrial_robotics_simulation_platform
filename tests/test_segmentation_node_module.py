import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

from test_utils import _setup_ros_stubs

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

    apm.msg.Detection2D = Detection2D
    apm.msg.Detection2DArray = Detection2DArray
    monkeypatch.setitem(sys.modules, 'apm_msgs', apm)
    monkeypatch.setitem(sys.modules, 'apm_msgs.msg', apm.msg)


class DummyBridge:
    def imgmsg_to_cv2(self, *a, **k):
        return np.zeros((4, 4, 3), dtype=np.uint8)

    def cv2_to_imgmsg(self, *a, **k):
        return types.SimpleNamespace(header=None)


def _setup_bridge(monkeypatch):
    import cv_bridge

    monkeypatch.setattr(cv_bridge, 'CvBridge', DummyBridge, raising=False)

    exec_mod = types.ModuleType('rclpy.executors')
    exec_mod.MultiThreadedExecutor = MagicMock()
    monkeypatch.setitem(sys.modules, 'rclpy.executors', exec_mod)


def test_load_config_defaults(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.segmentation_node', None)
    from advanced_perception import segmentation_node as sn

    node = sn.SegmentationNode()
    assert np.array_equal(node.threshold_min, np.array([0, 0, 0]))
    assert np.array_equal(node.threshold_max, np.array([255, 255, 255]))
    assert node.min_contour_area == 1000


def test_load_config_file(monkeypatch, tmp_path):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.segmentation_node', None)
    from advanced_perception import segmentation_node as sn

    cfg = tmp_path / 'seg.yaml'
    cfg.write_text(
        'threshold_min: [1, 2, 3]\nthreshold_max: [4, 5, 6]\nmin_contour_area: 50\n'
    )

    node = sn.SegmentationNode()
    node.segmentation_config_path = str(cfg)
    node.load_config()

    assert node.min_contour_area == 50
    assert node.threshold_min.tolist() == [1, 2, 3]
    assert node.threshold_max.tolist() == [4, 5, 6]


def test_process_image_publishes(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    _setup_apm(monkeypatch)
    _setup_bridge(monkeypatch)
    sys.modules.pop('advanced_perception.segmentation_node', None)
    from advanced_perception import segmentation_node as sn

    node = sn.SegmentationNode()
    det = sys.modules['apm_msgs'].msg.Detection2D()

    def fake_segment(image):
        return image, [det]

    monkeypatch.setattr(node, 'segment_image', fake_segment)

    msg = types.SimpleNamespace(header=object())
    node.process_image(msg)

    assert node.segmented_image_pub.publish.called
    assert node.segmented_objects_pub.publish.called
    out = node.segmented_objects_pub.publish.call_args[0][0]
    assert len(out.detections) == 1
