import importlib
import sys
from pathlib import Path

# Ensure packages under src/ are importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src'))
sys.path.append(str(ROOT / 'src' / 'robot_interfaces'))


def test_delta_interface_docstring():
    mod = importlib.import_module('robot_interfaces.delta_interface')
    assert mod.__doc__ == 'Delta robot specific interface helpers.'


def test_ur5_interface_docstring():
    mod = importlib.import_module('robot_interfaces.ur5_interface')
    assert mod.__doc__ == 'UR5 robot interface helpers.'
