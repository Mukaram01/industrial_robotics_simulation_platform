# Scenario Templates

This document shows minimal examples of scenario configuration files. Both YAML and JSON formats are supported and share the same keys.

## Example YAML

```yaml
# Based on src/simulation_core/config/sdf_example.yaml
description: Example configuration using an SDF model
environment:
  type: demo_world
  size: [5.0, 5.0, 3.0]
robots:
  - id: template_sdf_robot
    type: template
    model_file: ../../templates/robot_description/template_robot.sdf
```

## Equivalent JSON

```json
{
  "description": "Example configuration using an SDF model",
  "environment": {
    "type": "demo_world",
    "size": [5.0, 5.0, 3.0]
  },
  "robots": [
    {
      "id": "template_sdf_robot",
      "type": "template",
      "model_file": "../../templates/robot_description/template_robot.sdf"
    }
  ]
}
```

Use these snippets as a starting point when defining your own scenarios. Additional sections such as conveyors, containers and sorting rules may be added as shown in other files under `src/simulation_core/config`.

## Objects Example

Objects are listed under the `objects` key. Each entry requires an `id`,
`type`, `position` and `dimensions`. The `position` and `dimensions` arrays
contain three values representing meters. Set `pick_target: true` on any
object that should be grasped by a robot.

```yaml
objects:
  - id: support_table
    type: table
    position: [0.0, 0.0, 0.0]
    dimensions: [1.0, 1.0, 0.8]
  - id: part_1
    type: box
    position: [0.2, 0.0, 0.8]
    dimensions: [0.05, 0.05, 0.05]
    pick_target: true
```
