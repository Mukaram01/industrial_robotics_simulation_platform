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

## Notebook Usage

An interactive notebook is provided in `notebooks/benchmark_perception_demo.ipynb`.
Launch it from the repository root:

```bash
jupyter notebook notebooks/benchmark_perception_demo.ipynb
```

Edit the paths in the first code cell to point to your dataset. Running the
cells executes the benchmark and displays a bar chart of per-class AP values with
the overall mAP in the title.
