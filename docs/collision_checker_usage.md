# Collision Checker Usage

This page explains how to detect collisions programmatically using the
`collision_checker.detect_collisions` helper.

## Running the Function

The utility is part of the `simulation_core` package. Simply import the
function and pass your `environment_config` dictionary:

```python
from simulation_core import detect_collisions

env = {
    "objects": [
        {"id": "box1", "type": "box", "position": [0.0, 0.0, 0.0], "dimensions": [1, 1, 1]},
        {"id": "box2", "type": "box", "position": [0.4, 0.0, 0.0], "dimensions": [1, 1, 1]},
    ]
}

collisions = detect_collisions(env)
print(collisions)
```

This prints:

```python
[{"type": "collision", "objects": ["box1", "box2"]}]
```

Near misses can also be detected by providing `min_distance`:

```python
collisions = detect_collisions(env, min_distance=0.2)
```

## ROS2 Node

No dedicated collision checker node is provided. The `SafetyMonitorNode`
uses this helper internally to enforce safety rules.
