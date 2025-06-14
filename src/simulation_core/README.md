# simulation_core

Core nodes controlling the simulated environment and safety logic.

## Nodes
- `environment_configurator_node` – loads scenario files and publishes status/metrics.
- `safety_monitor_node` – checks safety rules and can trigger an emergency stop.
- `system_test_node` – basic integration test helper.

## Usage
Example of running the configurator:
```bash
ros2 run simulation_core environment_configurator_node
```

## Configuration
Scenario and safety YAML files live under `config/`. Pass `config_dir` to override the directory when launching.
