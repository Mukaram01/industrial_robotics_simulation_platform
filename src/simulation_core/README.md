# simulation_core

## Purpose
Core nodes controlling the simulated environment and safety logic.

## Setup
Build the package and source your workspace:

```bash
colcon build --packages-select simulation_core
source install/setup.bash
```

## Usage
Run the environment configurator:

```bash
ros2 run simulation_core environment_configurator_node
```

Use `simulation_tools/integrated_system_launch.py` to start all core services together.

## Extension
Scenario and safety YAML files live under `config/`. Pass `config_dir` when launching to use a custom directory or modify the provided files to create new scenarios.
