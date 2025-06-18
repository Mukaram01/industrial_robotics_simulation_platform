# apm_msgs

## Purpose
Message definitions used by the Advanced Perception Module and related packages.

## Setup
Build the workspace to generate the message headers:

```bash
colcon build --packages-select apm_msgs
source install/setup.bash
```

## Usage
Other packages depend on these message interfaces. No nodes are provided directly.

## Extension
The following messages are defined and can be extended as needed:
- `Detection2D` / `Detection2DArray`
- `DetectedObject` / `DetectedObjectArray`

Edit the files in `msg/` and rebuild to add new fields.
