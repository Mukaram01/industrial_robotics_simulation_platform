# industrial_protocols

OPC&nbsp;UA and MQTT bridge connecting the simulation to industrial systems.

## Nodes
- `industrial_protocol_bridge_node` – relays status and commands via OPC&nbsp;UA and MQTT.

## Usage
Run the bridge node:
```bash
ros2 run industrial_protocols industrial_protocol_bridge_node
```
Common parameters include `opcua_endpoint`, `mqtt_broker`, and `config_dir` for protocol configuration.
