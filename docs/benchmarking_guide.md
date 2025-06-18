# Benchmarking Guide

This guide explains how to evaluate the detection and pose estimation models
using the `benchmark_perception.py` script.

## Prerequisites

- Install the Python dependencies listed in `requirements.txt`.
- Ensure `onnxruntime`, `opencv-python` and `numpy` are available.
- Prepare a dataset in COCO format with images and a JSON annotation file.

## Running the Benchmark

From the repository root run:

```bash
python scripts/benchmark_perception.py \
  --images /path/to/images \
  --annotations /path/to/annotations.json \
  --output results.json
```

The script loads the ONNX detection model, processes every image in the dataset
and calculates mAP. Per-class AP values are also written to the specified output
file.

## Output

The output file is JSON formatted and contains the overall `mAP` along with the
average precision for each class. For example:

```json
{
  "mAP": 0.42,
  "AP_per_class": {
    "person": 0.55,
    "car": 0.36
  }
}
```

## Adjusting the Model

You can specify a different ONNX model with the `--model` option:

```bash
python scripts/benchmark_perception.py \
  --images /data/images \
  --annotations /data/annotations.json \
  --model path/to/model.onnx
```

## Planning Benchmark

The `benchmark_planning.py` script measures MoveIt planning performance.
It executes a series of named poses and records planning time, success
rate and average trajectory duration.

Run the planner benchmark with:

```bash
python scripts/benchmark_planning.py \
  --group manipulator \
  --poses home ready pick \
  --trials 5 \
  --output planning_results.json
```

The output JSON contains metrics for each pose:

```json
[
  {
    "pose": "home",
    "avg_planning_time": 1.2,
    "success_rate": 0.8,
    "avg_trajectory_duration": 3.4
  }
]
```
