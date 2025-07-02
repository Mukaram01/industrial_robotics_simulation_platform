# Industrial Robotics Simulation Platform - Executive Summary

[![Build](https://github.com/Mukaram01/industrial_robotics_simulation_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Mukaram01/industrial_robotics_simulation_platform/actions/workflows/ci.yml)
[![Tests](https://github.com/Mukaram01/industrial_robotics_simulation_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Mukaram01/industrial_robotics_simulation_platform/actions/workflows/ci.yml)

## Overview

The Industrial Robotics Simulation Platform is a comprehensive, highly configurable system designed for demonstrating, developing, and testing industrial robotics applications. This platform serves as both a powerful demonstration tool for showcasing capabilities to potential clients and a foundation for a commercial robotics company.

## Key Features

1. **Highly Dynamic Simulation Environment**
   - Configurable industrial scenarios (pick-and-place, sorting, quality inspection)
   - Physics-based simulation with collision detection using a modular AABB checker
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
- `industrial_protocols` – OPC UA/MQTT/Modbus integration
- `web_interface_backend` – Flask server and APIs
- `web_interface_frontend` – static HTML/JS resources
   - Performance metrics dashboard
   - 3D environment visualization
   - Matplotlib-based 3D object viewer
   - Live plots for joint angles and sensors

4. **Industrial Integration**
  - OPC UA, MQTT and Modbus protocol support
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

The UR5 meshes are distributed with the `ur_description` package and are not stored in this repository. See the [deployment guide](industrial_deployment_guide.md#ur5-mesh-assets) for instructions on installing the package and visualizing the UR5 model.

## Getting Started

For installation and launch instructions, see [industrial_deployment_guide.md](industrial_deployment_guide.md) and [docs/full_system_run_guide.md](docs/full_system_run_guide.md).

### Building the Workspace
From the repository root, build all packages via the meta-package:

```bash
colcon build --packages-select industrial_robotics_simulation_platform_meta
```

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
- Guide for running the test suite: [docs/testing_guide.md](docs/testing_guide.md)
- Collision checker usage: [docs/collision_checker_usage.md](docs/collision_checker_usage.md)
- RealSense camera usage: [docs/realsense_camera_guide.md](docs/realsense_camera_guide.md)
- See [CHANGELOG.md](CHANGELOG.md) for release history
- Benchmark planning and perception: [docs/benchmarking_guide.md](docs/benchmarking_guide.md)
- API documentation can be generated using Sphinx:
  ```bash
  cd docs/api
  make html
  ```
- Example SDF-based configuration: `src/simulation_core/config/sdf_example.yaml`
- Scenario template snippets: [docs/scenario_templates.md](docs/scenario_templates.md)
- Guide for running scripted experiments: [docs/experiment_runner.md](docs/experiment_runner.md)

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

## Running Experiments

Use `scripts/run_experiment.py` to launch the integrated system with
parameters defined in a YAML or JSON configuration file:

```bash
python scripts/run_experiment.py --config my_experiment.yaml
```

The script reads options such as `scenario`, `use_realsense` and
`use_advanced_perception` from the configuration file and forwards them
to the underlying ROS launch command.

## Simple Simulation Launch

`scripts/run_sim.py` offers a lightweight wrapper around the default
launch command. It starts the simulation with a few simple flags:

```bash
python scripts/run_sim.py --use-realsense
```

## Handling Large Files

Model weights and other large assets in the `models/` directory are stored using
[Git LFS](https://git-lfs.github.com/). Install Git LFS **before cloning** the
repository so these files download correctly:

```bash
sudo apt-get install git-lfs  # or follow the installer for your platform
git lfs install
git clone <repository-url>
```

This repository excludes large binary artifacts such as ONNX models and
compressed archives via `.gitignore`. If you need to keep these files under
version control, configure [Git LFS](https://git-lfs.github.com/) before
committing them:

```bash
git lfs install
git lfs track "*.onnx" "*.tar.gz" "*.ckpt"
git add .gitattributes
```

To revert any LFS-tracked file to its committed state, fetch the original file
and check it out:

```bash
git lfs fetch origin
git lfs checkout path/to/file
```

You can also discard local modifications with `git restore path/to/file`.

## Ignored Paths

Temporary data and logs are not stored in version control. The `data/` folder is
used for runtime outputs, while scripts may create configuration files under
`configs/`. These locations are listed in `.gitignore` so your local runs don't
pollute commits.

## Docker Usage

Build the image from the repository root:

```bash
docker build -t industrial_sim .
```

Run the container with host networking so ROS 2 topics are visible:

```bash
docker run --rm -it --net=host industrial_sim
```

The container will automatically launch the integrated system using
`simulation_tools/integrated_system_launch.py`.

## Running Tests

`pytest` is used for running the unit tests. After building the workspace,
Python 3.10 is recommended when executing the test suite. Install any Python dependencies with:

```bash
pip install -r requirements.txt
```

Then execute:

```bash
pytest
```

You can also run the `setup_and_test.sh` helper script, which installs the
dependencies and executes `flake8` followed by `pytest`:

```bash
scripts/setup_and_test.sh
```

### Tests Directory

All unit tests are located in the `tests/` folder. Each file targets a specific
package or ROS2 node and uses lightweight stub modules so the suite can run
without a full ROS installation. After installing the dependencies, run
`pytest` from the repository root to execute all tests. When adding new
functionality, place the corresponding test cases in this directory.

### Code Style

`flake8` checks are provided to help maintain consistent code formatting.
Install all development dependencies and run the style checker before submitting changes:

```bash
pip install -r requirements.txt
flake8 src tests
```

## License

All packages in this repository are released under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file and each package's `package.xml` or `LICENSE` file for details.
