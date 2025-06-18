# robot_interfaces

## Purpose
Python modules that implement common interfaces for different robot types.

## Setup
Install the package into your workspace:

```bash
colcon build --packages-select robot_interfaces
source install/setup.bash
```

## Usage
This package provides library modules only; no nodes are launched directly.

## Extension
Currently skeleton interfaces for the delta robot and the UR5 are provided. Extend the modules under `robot_interfaces/` to support additional robots.
