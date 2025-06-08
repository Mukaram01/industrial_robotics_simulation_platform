# Industrial Robotics Simulation Platform - Executive Summary

## Overview

The Industrial Robotics Simulation Platform is a comprehensive, highly configurable system designed for demonstrating, developing, and testing industrial robotics applications. This platform serves as both a powerful demonstration tool for showcasing capabilities to potential clients and a foundation for a commercial robotics company.

## Key Features

1. **Highly Dynamic Simulation Environment**
   - Configurable industrial scenarios (pick-and-place, sorting, quality inspection)
   - Physics-based simulation with collision detection
   - Support for multiple robot types (delta, articulated arms)

2. **Interactive Web-Based GUI**
   - Real-time monitoring and control
   - Dynamic object manipulation
   - Scenario configuration and management

3. **Advanced Visualization**
   - Multi-view camera perspectives
   - Performance metrics dashboard
   - 3D environment visualization

4. **Industrial Integration**
   - OPC UA and MQTT protocol support
   - Hybrid mode for connecting to real equipment
   - Pre-trained model integration

5. **Safety and Compliance**
   - Comprehensive safety monitoring
   - Emergency stop functionality
   - Safety zone enforcement

6. **Demonstration Tools**
  - Scenario recording and playback
  - Error simulation for robustness testing
  - Data export for analysis and reporting

## UR5 Mesh Assets

The UR5 mesh files used by `ur5_robot_description` are automatically downloaded
or bundled with this repository. No manual download or CMake modification is
required. Once dependencies are installed, build the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Visualizing the UR5

Once the meshes are in place, you can visualize the UR5 model in RViz:

```bash
ros2 launch ur5_robot_description display.launch.py
```

To start MoveIt for the UR5, run:

```bash
ros2 launch ur5_robot_moveit_config move_group.launch.py use_sim_time:=false
```

The mesh resources are required for accurate visualization in both cases.

## Getting Started

1. **Installation**
   ```bash
   source /opt/ros/humble/setup.bash
   cd industrial_robotics_simulation_platform
   pip install -r requirements.txt
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch the System**
   ```bash
   ros2 launch simulation_tools integrated_system_launch.py
   ```

   To load a specific scenario or enable advanced perception:
   ```bash
   ros2 launch simulation_tools integrated_system_launch.py \
       scenario:=warehouse use_advanced_perception:=true
   ```
   The `scenario` argument defaults to `default`.

   To change the OPC UA server port, pass the `opcua_port` argument:
   ```bash
  ros2 launch simulation_tools integrated_system_launch.py opcua_port:=4841
  ```
  This argument is also supported by `realsense_hybrid_launch.py`.

   To secure MQTT communication, configure authentication and TLS in your
   Mosquitto broker. Set the `password_file` option and create a dedicated
   listener for TLS traffic:
   ```
   listener 8883
   password_file /etc/mosquitto/passwd
   cafile /etc/mosquitto/ca.crt
   certfile /etc/mosquitto/server.crt
   keyfile /etc/mosquitto/server.key
   allow_anonymous false
   ```
   Refer to the [Mosquitto security documentation](https://mosquitto.org/man/mosquitto-conf-5.html) for details.

   The OPC UA server is configured with minimal security and, by default, only
   listens on `127.0.0.1`. If you need to allow remote connections, override the
   `opcua_endpoint` parameter with a host accessible on your network, e.g.:
   ```bash
   ros2 launch simulation_tools integrated_system_launch.py \
       opcua_endpoint:=opc.tcp://0.0.0.0:4840/freeopcua/server/
   ```

3. **Access the Web Interface**
   ```
   http://localhost:8080
   ```

### Web Interface Configuration

The web server uses Flask's development server via `socketio.run`. By default,
the system allows Werkzeug to run even if a production environment is
detected. You can disable this override by setting the `allow_unsafe_werkzeug`
parameter to `false`:

```bash
ros2 launch simulation_tools integrated_system_launch.py allow_unsafe_werkzeug:=false
```

Both `web_interface_node` and `visualization_server_node` support a `jpeg_quality`
parameter to control JPEG compression (0-100). The default value is `75`.

## Documentation

For complete details, please refer to the included `industrial_deployment_guide.md` which provides comprehensive instructions for:
- System configuration
- Industrial use case setup
- Hybrid mode operation
- Advanced feature usage
- Customization options
- Troubleshooting
- API reference

## Commercial Applications

This platform is designed to serve as the foundation for a robotics company, with:
- Scalable architecture for commercial deployment
- Professional-grade industrial protocol support
- Extensible framework for proprietary algorithms
- Seamless transition from simulation to real hardware

## Next Steps

1. Review the deployment guide for detailed usage instructions
2. Explore the included industrial scenarios
3. Customize the environment for your specific demonstration needs
4. Consider hardware integration options for hybrid operation

## Running Tests

`pytest` is used for running the unit tests. After building the workspace,
install any Python dependencies and execute:

```bash
pytest
```

## License

All packages in this repository are released under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file and each package's `package.xml` or `LICENSE` file for details.
