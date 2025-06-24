# Performance Profiling Guide

This guide explains how to measure execution time for the simulation loop and perception nodes using the utilities in `tools/profiling`.

## 1. Install Optional Dependencies

The examples use Python's built-in `cProfile` module. For more detailed reports you can install `pyinstrument`:

```bash
pip install pyinstrument
```

## 2. Profile the Simulation Loop

Run the helper script to execute a dummy simulation loop under the profiler:

```bash
python tools/profiling/profile_simulation.py --steps 5000
```

The results are written to `simulation_profile.prof`. Open the file with the `pstats` module or a viewer such as `snakeviz`.

To use `pyinstrument` instead:

```bash
python tools/profiling/profile_simulation.py --profiler pyinstrument
```

## 3. Profile a Perception Pipeline

The perception example processes synthetic images:

```bash
python tools/profiling/profile_perception.py --images 200
```

Specify `--profiler pyinstrument` to use `pyinstrument` if installed. Results are saved to `perception_profile.prof`.

Both scripts are minimal templates. Replace the dummy functions with calls into your actual simulation or perception nodes to analyse their performance.

## 4. Profile Planning Nodes and System-Wide Performance

Use the `benchmark_planning.py` script to measure latency in the planning stack while gathering CPU and memory statistics for the entire system. The script executes a planning scenario multiple times and records aggregated metrics.

```bash
python scripts/benchmark_planning.py \
  --scenario configs/pick_and_place.yaml
```

Increase the number of iterations to stress test the planner and capture system-wide behaviour:

```bash
python scripts/benchmark_planning.py \
  --scenario configs/pick_and_place.yaml \
  --runs 50 --system-stats
```

Results are written to `planning_profile.json` for further analysis alongside the simulation and perception profiles.

## 5. Error Simulation Rate

The simulation includes optional error injection controlled by the
`error_simulation_rate` parameter. It defines the probability of triggering a
random error whenever metrics are published. Acceptable values range from `0.0`
(no errors) to `1.0` (an error every cycle).

Set the parameter when launching the system:

```bash
ros2 launch simulation_core full_system.launch.py error_simulation_rate:=0.3
```

You can adjust the rate while the system is running using the same configuration
topic described in the full system guide.
