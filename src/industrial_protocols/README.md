# industrial_protocols

## Purpose
OPC&nbsp;UA, MQTT and Modbus bridge connecting the simulation to industrial systems.

## Setup
Build the package and source the workspace:

```bash
colcon build --packages-select industrial_protocols
source install/setup.bash
```

## Usage
Run the bridge node:

```bash
ros2 run industrial_protocols industrial_protocol_bridge_node
```

## Extension
Tune connection parameters using the following arguments: `opcua_endpoint`, `mqtt_broker`, `modbus_host`, `modbus_port` and `config_dir` containing protocol configuration files.
