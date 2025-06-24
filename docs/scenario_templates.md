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
