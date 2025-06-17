#!/usr/bin/env python3
"""Utility that downloads a TensorFlow model and converts it to ONNX format."""

import os
import argparse
import tensorflow as tf
import tf2onnx
import numpy as np
from pathlib import Path
import tarfile
import urllib.request

def download_tf_model(model_name, output_dir):
    """
    Download a TensorFlow model from TensorFlow Hub or Model Zoo
    """
    print(f"Downloading TensorFlow model: {model_name}")
    
    # Map of supported models and their download URLs
    model_map = {
        "ssd_mobilenet_v2": "http://download.tensorflow.org/models/object_detection/tf2/20200711/ssd_mobilenet_v2_320x320_coco17_tpu-8.tar.gz",
        "efficientdet_d0": "http://download.tensorflow.org/models/object_detection/tf2/20200711/efficientdet_d0_coco17_tpu-32.tar.gz",
        "faster_rcnn_resnet50": "http://download.tensorflow.org/models/object_detection/tf2/20200711/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8.tar.gz"
    }
    
    if model_name not in model_map:
        raise ValueError(f"Model {model_name} not supported. Available models: {list(model_map.keys())}")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Download and extract the model
    model_url = model_map[model_name]
    model_file = os.path.join(output_dir, f"{model_name}.tar.gz")
    
    # Download the model file if it doesn't exist
    if not os.path.exists(model_file):
        try:
            print(f"Downloading {model_url} to {model_file}")
            urllib.request.urlretrieve(model_url, model_file)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to download model from {model_url}"
            ) from exc
    
    # Extract the model if the directory doesn't exist
    extract_dir = os.path.join(output_dir, model_name)
    if not os.path.exists(extract_dir):
        os.makedirs(extract_dir, exist_ok=True)
        with tarfile.open(model_file) as tar:
            tar.extractall(path=extract_dir)
    
    # Return the path to the saved model directory
    saved_model_dir = os.path.join(extract_dir, "saved_model")
    return saved_model_dir

def convert_tf_to_onnx(saved_model_dir, output_path, opset=13):
    """
    Convert TensorFlow SavedModel to ONNX format using from_saved_model
    """
    print(f"Converting TensorFlow model to ONNX: {saved_model_dir}")
    
    # Create parent directory if it doesn't exist
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Use the recommended approach for TensorFlow 2.x models
    model_proto, external_tensor_storage = tf2onnx.convert.from_saved_model(
        saved_model_dir,
        output_path=str(output_path),
        opset=opset,
        input_names=None,  # Auto-detect input names
        output_names=None  # Auto-detect output names
    )
    
    print(f"ONNX model saved to: {output_path}")
    return str(output_path)

def main():
    parser = argparse.ArgumentParser(description='Download and convert TensorFlow models to ONNX format')
    parser.add_argument('--model', type=str, default='ssd_mobilenet_v2', 
                        help='Model name (ssd_mobilenet_v2, efficientdet_d0, faster_rcnn_resnet50)')
    parser.add_argument('--output_dir', type=str, default='models',
                        help='Output directory for downloaded and converted models')
    parser.add_argument('--opset', type=int, default=13,
                        help='ONNX opset version')
    
    args = parser.parse_args()
    
    # Download TensorFlow model
    saved_model_dir = download_tf_model(args.model, args.output_dir)
    
    # Convert to ONNX
    onnx_path = os.path.join(args.output_dir, f"{args.model}.onnx")
    convert_tf_to_onnx(saved_model_dir, onnx_path, args.opset)
    
    print("Model conversion completed successfully!")

if __name__ == "__main__":
    main()
