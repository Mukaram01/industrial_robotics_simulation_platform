"""Discovery registry for robot and controller plugins."""

from importlib import metadata
from typing import Any, Dict, Type

_robot_classes: Dict[str, Type[Any]] = {}
_controller_classes: Dict[str, Type[Any]] = {}
_discovered = False


def _discover_entrypoints() -> None:
    global _discovered
    if _discovered:
        return

    eps = metadata.entry_points()
    for ep in eps.select(group="simulation_core.robots"):
        try:
            cls = ep.load()
        except Exception:
            continue
        _robot_classes.setdefault(ep.name, cls)
    for ep in eps.select(group="simulation_core.controllers"):
        try:
            cls = ep.load()
        except Exception:
            continue
        _controller_classes.setdefault(ep.name, cls)
    _discovered = True


def get_robot(name: str):
    """Return registered robot class by name or None."""
    _discover_entrypoints()
    return _robot_classes.get(name)


def get_controller(name: str):
    """Return registered controller class by name or None."""
    _discover_entrypoints()
    return _controller_classes.get(name)


def register_robot(name: str, cls: Type[Any]) -> None:
    """Manually register a robot class."""
    _robot_classes[name] = cls


def register_controller(name: str, cls: Type[Any]) -> None:
    """Manually register a controller class."""
    _controller_classes[name] = cls


class BasicController:
    """Default no-op controller used when none is specified."""


register_controller("basic", BasicController)
