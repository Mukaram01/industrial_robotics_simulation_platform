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
