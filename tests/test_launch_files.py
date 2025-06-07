import sys
from pathlib import Path

import pytest

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))

launch = pytest.importorskip('launch')
launch_ros = pytest.importorskip('launch_ros')


def _collect_node_names(description):
    names = []
    for action in description.entities:
        name = getattr(action, 'name', None)
        if isinstance(name, str):
            names.append(name)
    return names


def test_integrated_system_launch_realsense_disabled(monkeypatch):
    from simulation_tools.launch import integrated_system_launch

    monkeypatch.setattr(integrated_system_launch, 'LaunchConfiguration', lambda *args, **kwargs: 'false')
    ld = integrated_system_launch.generate_launch_description()
    names = _collect_node_names(ld)
    assert 'realsense2_camera' not in names
    assert 'camera_simulator' in names


def test_integrated_system_launch_realsense_enabled(monkeypatch):
    from simulation_tools.launch import integrated_system_launch

    monkeypatch.setattr(integrated_system_launch, 'LaunchConfiguration', lambda *args, **kwargs: 'true')
    ld = integrated_system_launch.generate_launch_description()
    names = _collect_node_names(ld)
    assert 'realsense2_camera' in names
    assert 'camera_simulator' not in names


def test_realsense_hybrid_launch_has_realsense():
    from simulation_tools.launch import realsense_hybrid_launch

    ld = realsense_hybrid_launch.generate_launch_description()
    names = _collect_node_names(ld)
    assert 'realsense2_camera' in names
