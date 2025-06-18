"""Utility functions for basic AABB collision detection."""

from typing import Dict, Iterable, List, Optional


def build_aabb(obj: Dict) -> Optional[Dict]:
    """Return an axis-aligned bounding box for an environment object."""
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


def detect_collisions(objects: Iterable[Dict], min_distance: float = 0.0) -> List[Dict]:
    """Return a list of collision or near-miss violations."""
    objs = list(objects)
    violations: List[Dict] = []

    for i in range(len(objs)):
        a = objs[i]
        for j in range(i + 1, len(objs)):
            b = objs[j]
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
                violations.append(
                    {"type": "collision", "objects": [a["id"], b["id"],], "distance": dist}
                )

    return violations
