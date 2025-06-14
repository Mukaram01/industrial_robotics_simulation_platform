# fmm_core

Flexible Manipulation Module providing MoveIt based manipulation nodes.

## Nodes
- `fmm_moveit_interface_node` – helper interface to MoveIt.
- `planning_scene_updater_node` – populates the planning scene with detected objects.
- `pick_and_place_node` – executes pick and place routines.
- `sorting_demo_node` – example sorting workflow.

## Launch files
- `fmm_moveit_system_launch.py` – brings up MoveIt together with perception.
- `pick_and_place_launch.py` – runs the pick and place node.
- `sorting_demo_launch.py` – example sorting setup.
- `load_sorting_config_launch.py` – utility to load sorting parameters.

## Usage
Run the pick and place demo:
```bash
ros2 launch fmm_core pick_and_place_launch.py
```

## Configuration
Sorting categories and robot parameters are stored in `config/default_sorting_config.yaml`.
