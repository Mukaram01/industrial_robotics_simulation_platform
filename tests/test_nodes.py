import sys
from pathlib import Path

import pytest

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))

rclpy = pytest.importorskip('rclpy')


def _init_node(node_cls):
    rclpy.init(args=None)
    node = node_cls()
    try:
        return node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_segmentation_node_defaults():
    pytest.importorskip('cv2')
    pytest.importorskip('cv_bridge')
    from advanced_perception.segmentation_node import SegmentationNode
    node = _init_node(SegmentationNode)
    assert node.get_parameter('camera_topic').value == '/camera/color/image_raw'
    assert node.get_parameter('segmentation_config').value == ''


def test_pose_estimation_node_defaults():
    pytest.importorskip('cv2')
    pytest.importorskip('cv_bridge')
    from advanced_perception.pose_estimation_node import PoseEstimationNode
    node = _init_node(PoseEstimationNode)
    assert node.get_parameter('camera_topic').value == '/camera/color/image_raw'
    assert node.get_parameter('depth_topic').value == '/camera/depth/image_rect_raw'
    assert node.get_parameter('camera_info_topic').value == '/camera/color/camera_info'
    assert node.get_parameter('pose_estimation_config').value == ''


def test_environment_configurator_node_defaults():
    from simulation_tools.environment_configurator_node import EnvironmentConfiguratorNode
    node = _init_node(EnvironmentConfiguratorNode)
    assert node.get_parameter('default_scenario').value == 'default'
    assert node.get_parameter('physics_enabled').value is True
    assert node.get_parameter('record_metrics').value is True


def test_safety_monitor_node_defaults():
    from simulation_tools.safety_monitor_node import SafetyMonitorNode
    node = _init_node(SafetyMonitorNode)
    assert node.get_parameter('safety_rules_file').value == 'safety_rules.yaml'
    assert node.get_parameter('emergency_stop_enabled').value is True
    assert node.get_parameter('collision_detection_enabled').value is True


def test_web_interface_node_defaults():
    from simulation_tools.web_interface_node import WebInterfaceNode
    node = _init_node(WebInterfaceNode)
    assert node.get_parameter('allow_unsafe_werkzeug').value is False
