# industrial_protocols

OPC&nbsp;UA, MQTT and Modbus bridge connecting the simulation to industrial systems.

## Nodes
- `industrial_protocol_bridge_node` – relays status and commands via OPC&nbsp;UA, MQTT and Modbus.

## Usage
Run the bridge node:
```bash
ros2 run industrial_protocols industrial_protocol_bridge_node
```
Common parameters include `opcua_endpoint`, `mqtt_broker`, `modbus_host`, `modbus_port`, and `config_dir` for protocol configuration.
