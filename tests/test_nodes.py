import sys
from pathlib import Path

import pytest

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'web_interface_backend'))
sys.path.append(str(ROOT / 'src' / 'simulation_core'))

rclpy = pytest.importorskip('rclpy')
apm_msgs = pytest.importorskip("apm_msgs")


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
    assert (
        node.get_parameter('depth_topic').value
        == '/camera/depth/image_rect_raw'
    )
    assert (
        node.get_parameter('camera_info_topic').value
        == '/camera/color/camera_info'
    )
    assert node.get_parameter('pose_estimation_config').value == ''


def test_environment_configurator_node_defaults():
    from simulation_core.environment_configurator_node import (
        EnvironmentConfiguratorNode,
    )
    node = _init_node(EnvironmentConfiguratorNode)
    assert node.get_parameter('default_scenario').value == 'default'
    assert node.get_parameter('scenario').value == ''
    assert node.get_parameter('physics_enabled').value is True
    assert node.get_parameter('record_metrics').value is True


def test_safety_monitor_node_defaults():
    from simulation_core.safety_monitor_node import SafetyMonitorNode
    node = _init_node(SafetyMonitorNode)
    assert node.get_parameter('safety_rules_file').value == 'safety_rules.yaml'
    assert node.get_parameter('emergency_stop_enabled').value is True
    assert node.get_parameter('collision_detection_enabled').value is True


def test_web_interface_node_defaults():
    import os
    os.environ["WEB_INTERFACE_SECRET"] = "dummy_key"
    from web_interface_backend.web_interface_node import WebInterfaceNode
    node = _init_node(WebInterfaceNode)
    assert node.get_parameter('allow_unsafe_werkzeug').value is True
    assert node.get_parameter('save_images').value is False
    assert node.get_parameter('log_db_path').value == ''
    assert node.get_parameter('jpeg_quality').value == 75
    assert (
        node.get_parameter('detected_objects_topic').value
        == '/apm/detection/objects'
    )
    assert node.get_parameter('secret_key').value == 'dummy_key'


def test_visualization_server_node_defaults():
    from web_interface_backend.visualization_server_node import VisualizationServerNode
    node = _init_node(VisualizationServerNode)
    assert node.get_parameter('data_dir').value == ''
    assert node.get_parameter('export_enabled').value is True
    assert node.get_parameter('export_interval').value == 60.0
    assert node.get_parameter('visualization_rate').value == 10.0
    assert node.get_parameter('jpeg_quality').value == 75


def test_industrial_protocol_bridge_defaults(monkeypatch):
    from test_utils import _setup_ros_stubs

    _setup_ros_stubs(monkeypatch)
    sys.modules.pop('industrial_protocols.industrial_protocol_bridge_node', None)
    from industrial_protocols import industrial_protocol_bridge_node as ipb

    node = ipb.IndustrialProtocolBridgeNode()

    assert node.get_parameter('modbus_host').value == 'localhost'
    assert node.get_parameter('modbus_port').value == 502
    assert node.get_parameter('modbus_enabled').value is False
