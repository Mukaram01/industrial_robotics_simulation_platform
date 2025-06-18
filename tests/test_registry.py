import sys
from types import ModuleType

import importlib.metadata as md

sys.path.append('src')
sys.path.append('src/simulation_core')

from simulation_core import registry  # noqa: E402


def test_entrypoint_discovery(monkeypatch):
    plugin_mod = ModuleType('plugin_mod')

    class MyRobot:
        pass

    class MyController:
        pass

    plugin_mod.MyRobot = MyRobot
    plugin_mod.MyController = MyController
    sys.modules['plugin_mod'] = plugin_mod

    ep_robot = md.EntryPoint(
        name='my_robot', value='plugin_mod:MyRobot', group='simulation_core.robots'
    )
    ep_controller = md.EntryPoint(
        name='my_controller', value='plugin_mod:MyController', group='simulation_core.controllers'
    )
    eps = md.EntryPoints([ep_robot, ep_controller])
    monkeypatch.setattr(md, 'entry_points', lambda: eps)

    registry._robot_classes.clear()
    registry._controller_classes.clear()
    registry._discovered = False

    assert registry.get_robot('my_robot') is MyRobot
    assert registry.get_controller('my_controller') is MyController

    registry.register_controller('basic', registry.BasicController)
    sys.modules.pop('plugin_mod')
