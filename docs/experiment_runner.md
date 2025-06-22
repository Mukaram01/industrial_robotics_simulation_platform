# Experiment Runner Guide

This document explains how to use the `run_experiment.py` script to launch reproducible simulation experiments.

## Usage

Run the script from the repository root and pass a YAML or JSON configuration file:

```bash
python scripts/run_experiment.py --config my_experiment.yaml
```

The script parses the file and constructs a `ros2 launch` command for `simulation_tools/integrated_system_launch.py`.
All recognized keys map directly to the launch arguments used by `integrated_system_launch.py`.

## Sample Configuration

```yaml
use_realsense: false
use_advanced_perception: true
scenario: pick_and_place
config_dir: configs/demo
data_dir: /tmp/sim_data
save_images: true
allow_unsafe_werkzeug: true
secret_key: mysecret
opcua_port: 4840
```

An equivalent JSON file may be used with the same keys.

## Script Behaviour

`run_experiment.py` reads the configuration, converts boolean values to the strings expected by ROS 2 launch and invokes:

```bash
ros2 launch simulation_tools integrated_system_launch.py \
  use_realsense:=<true|false> \
  use_advanced_perception:=<true|false> \
  scenario:=<scenario> \
  config_dir:=<config_dir> \
  data_dir:=<data_dir> \
  save_images:=<true|false> \
  allow_unsafe_werkzeug:=<true|false> \
  secret_key:=<key> \
  opcua_port:=<port>
```

The secret key may also be provided via the `WEB_INTERFACE_SECRET`
environment variable.

Any unknown keys in the configuration file are ignored.
