# fmm_core

## Purpose
Flexible Manipulation Module providing MoveIt based manipulation nodes.

## Setup
Build this package and source the workspace:

```bash
colcon build --packages-select fmm_core
source install/setup.bash
```

## Usage
Run the pick and place demo:

```bash
ros2 launch fmm_core pick_and_place_launch.py
```

Other launch files include:
- `fmm_moveit_system_launch.py` – brings up MoveIt together with perception.
- `sorting_demo_launch.py` – example sorting setup.
- `load_sorting_config_launch.py` – utility to load sorting parameters.

## Extension
Sorting categories and robot parameters are stored in `config/default_sorting_config.yaml`. Adjust this file or provide a custom configuration to change the demo behaviour.
