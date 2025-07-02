import sys
from pathlib import Path
from test_utils import _setup_ros_stubs

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))

def test_basic_collision(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    from simulation_core.collision_checker import build_aabb, detect_collisions

    a = {'id': 'a', 'type': 'box', 'position': [0.0, 0.0, 0.0], 'dimensions': [1, 1, 1]}
    b = {'id': 'b', 'type': 'box', 'position': [0.4, 0.0, 0.0], 'dimensions': [1, 1, 1]}
    aabbs = [build_aabb(a), build_aabb(b)]
    out = detect_collisions(aabbs)
    assert out
    assert out[0]['type'] == 'collision'


def test_near_miss_detection(monkeypatch):
    _setup_ros_stubs(monkeypatch)
    from simulation_core.collision_checker import build_aabb, detect_collisions

    a = {'id': 'a', 'type': 'box', 'position': [0.0, 0.0, 0.0], 'dimensions': [1, 1, 1]}
    b = {'id': 'b', 'type': 'box', 'position': [1.2, 0.0, 0.0], 'dimensions': [1, 1, 1]}
    aabbs = [build_aabb(a), build_aabb(b)]
    # No violation without min_distance
    assert detect_collisions(aabbs) == []
    near = detect_collisions(aabbs, min_distance=0.5)
    assert near
    assert near[0]['type'] == 'near_miss'
    assert 'distance' in near[0]
