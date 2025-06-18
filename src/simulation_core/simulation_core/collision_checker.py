"""Utility functions for simple AABB collision detection."""

from __future__ import annotations

from typing import Any, Dict, List


_AABB = Dict[str, Any]


def _to_aabb(obj: Dict[str, Any]) -> _AABB | None:
    pos = obj.get("position")
    dims = obj.get("dimensions")
    if pos is None:
        return None
    if dims is None:
        radius = obj.get("parameters", {}).get("workspace_radius")
        if radius is not None:
            dims = [2 * radius, 2 * radius, 2 * radius]
    if dims is None:
        return None
    half = [d / 2.0 for d in dims]
    return {
        "id": obj.get("id", obj.get("type", "object")),
        "min": [pos[i] - half[i] for i in range(3)],
        "max": [pos[i] + half[i] for i in range(3)],
    }


def detect_collisions(environment_config: Dict[str, Any], min_distance: float = 0.0) -> List[Dict[str, Any]]:
    """Return a list of collisions or near misses between objects.

    Parameters
    ----------
    environment_config:
        Dictionary containing lists of objects under the keys
        ``objects``, ``containers``, ``conveyors`` and ``robots``.
    min_distance:
        Minimum allowed distance between objects. If the distance is
        below this value a near miss violation is reported.
    """
    objects: List[_AABB] = []
    for key in ["objects", "containers", "conveyors", "robots"]:
        for obj in environment_config.get(key, []):
            aabb = _to_aabb(obj)
            if aabb:
                objects.append(aabb)

    violations: List[Dict[str, Any]] = []
    for i in range(len(objects)):
        a = objects[i]
        for j in range(i + 1, len(objects)):
            b = objects[j]
            overlap = all(
                a["min"][k] <= b["max"][k] and a["max"][k] >= b["min"][k]
                for k in range(3)
            )
            if overlap:
                violations.append({"type": "collision", "objects": [a["id"], b["id"]]})
                continue

            dx = max(a["min"][0] - b["max"][0], b["min"][0] - a["max"][0], 0.0)
            dy = max(a["min"][1] - b["max"][1], b["min"][1] - a["max"][1], 0.0)
            dz = max(a["min"][2] - b["max"][2], b["min"][2] - a["max"][2], 0.0)
            dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
            if dist < min_distance:
                violations.append({"type": "collision", "objects": [a["id"], b["id"]], "distance": dist})

    return violations
