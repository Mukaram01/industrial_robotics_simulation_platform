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

### Package Structure
The ROS workspace is organized into modular packages:

- `simulation_core` – base simulation logic and scenarios
- `robot_interfaces` – robot-specific communication layers
- `perception_nodes` – camera simulation and sensor nodes
- `industrial_protocols` – OPC UA/MQTT integration
- `web_interface_backend` – Flask server and APIs
- `web_interface_frontend` – static HTML/JS resources
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

The UR5 mesh files used by `ur5_robot_description` are not stored directly in
this repository. Install the `ur_description` package for your ROS&nbsp;2
distribution so that these meshes are available. On Ubuntu you can install the
package with:

```bash
sudo apt-get install ros-<distro>-ur-description
```

The mesh files will then be located under
`/opt/ros/<distro>/share/ur_description/meshes`. If you build
`ur_description` from source or place the meshes elsewhere, make sure the path
is reachable via `ROS_PACKAGE_PATH` or copy the meshes into
`src/ur5_robot_description/meshes`.

Once the package is installed and other dependencies are available, build the
workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Visualizing the UR5

Once the meshes are in place, you can visualize the UR5 model in RViz. Running
the following command now starts RViz automatically with a default
configuration:

```bash
ros2 launch ur5_robot_description display.launch.py
```

To start MoveIt for the UR5, run:

```bash
ros2 launch ur5_robot_moveit_config move_group.launch.py use_sim_time:=false
```

The mesh resources are required for accurate visualization in both cases.

- If RViz opens without showing the robot, confirm you have a working graphical desktop and that the workspace has been built and sourced.
- The launch file now passes the URDF to RViz directly so the model also loads correctly when a namespace is specified, e.g. `ros2 launch ur5_robot_description display.launch.py --ros-args -r __ns:=my_robot`.

## Getting Started

1. **Installation**
   ```bash
   source /opt/ros/humble/setup.bash
   cd industrial_robotics_simulation_platform
   pip install -r requirements.txt
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src -y --ignore-src
   colcon build --symlink-install
   source install/setup.bash
   ```
   The `rosdep` commands resolve additional ROS packages required for the workspace.

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

   The industrial protocol bridge expects a local MQTT broker. Install and start
   Mosquitto with:

   ```bash
   sudo apt install mosquitto
   sudo systemctl start mosquitto
   ```

   Launch with `mqtt_enabled:=false` if a broker is not available.

   The OPC UA server is configured with minimal security and, by default, only
   listens on `127.0.0.1`. If you need to allow remote connections, override the
   `opcua_endpoint` parameter with a host accessible on your network, e.g.:
   ```bash
   ros2 launch simulation_tools integrated_system_launch.py \
       opcua_endpoint:=opc.tcp://0.0.0.0:4840/freeopcua/server/
   ```

### Configuration Directory and Data Storage

By default, configuration files are loaded from
`src/simulation_core/config/`. Pass `config_dir:=<path>` when launching to
use a custom directory. The `data_dir` parameter controls where log files and
optional saved images are written (default: `/tmp/simulation_data`).

Example:

```bash
ros2 launch simulation_tools integrated_system_launch.py \
    config_dir:=/my/configs data_dir:=/tmp/my_data
```

Scenario YAML files such as `pick_and_place.yaml` can be duplicated and
modified in your custom directory to define new scenarios.

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

Both nodes in the `web_interface_backend` package, `web_interface_node` and
`visualization_server_node`, support a `jpeg_quality` parameter to control JPEG
compression (0-100). The default value is `75`.

Set `auto_open_browser:=true` to automatically open your default web browser
when the interface starts. This is disabled by default for headless systems.

## Documentation

For complete details, please refer to the included `industrial_deployment_guide.md` which provides comprehensive instructions for:
- System configuration
- Industrial use case setup
- Hybrid mode operation
- Advanced feature usage
- Customization options
- Troubleshooting
- API reference
- Step-by-step instructions for running the full system: [docs/full_system_run_guide.md](docs/full_system_run_guide.md)
- Guide for integrating new robots: [docs/robot_integration_guide.md](docs/robot_integration_guide.md)

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

## Handling Large Files

This repository excludes large binary artifacts such as ONNX models and
compressed archives via `.gitignore`. If you need to keep these files under
version control, configure [Git LFS](https://git-lfs.github.com/) before
committing them:

```bash
git lfs install
git lfs track "*.onnx" "*.tar.gz" "*.ckpt"
git add .gitattributes
```

## Running Tests

`pytest` is used for running the unit tests. After building the workspace,
install any Python dependencies and execute:

```bash
pytest
```

### Tests Directory

All unit tests are located in the `tests/` folder. Each file targets a specific
package or ROS2 node and uses lightweight stub modules so the suite can run
without a full ROS installation. After installing the dependencies, run
`pytest` from the repository root to execute all tests. When adding new
functionality, place the corresponding test cases in this directory.

### Code Style

`flake8` checks are provided to help maintain consistent code formatting.
Install the tool and run it against the source tree before submitting changes:

```bash
pip install flake8
flake8 src tests
```

## License

All packages in this repository are released under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file and each package's `package.xml` or `LICENSE` file for details.
