#!/usr/bin/env python3
"""Benchmark detection and pose estimation models on a dataset."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def preprocess_image(image, input_size):
    """Resize and normalize an image for the model."""
    resized = cv2.resize(image, input_size)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 255.0
    return np.expand_dims(normalized, axis=0)


def process_detections(outputs, image_shape, class_map, conf_threshold=0.5,
                       max_detections=100):
    """Convert model outputs into structured detections."""
    boxes = outputs[0][0]
    scores = outputs[1][0]
    classes = outputs[2][0]

    detections = []
    height, width = image_shape[:2]
    for i in range(min(len(scores), max_detections)):
        if scores[i] >= conf_threshold:
            class_id = int(classes[i])
            class_name = class_map.get(class_id, str(class_id))
            box = boxes[i]
            x1 = int(box[1] * width)
            y1 = int(box[0] * height)
            x2 = int(box[3] * width)
            y2 = int(box[2] * height)
            detections.append({
                "class_id": class_id,
                "class_name": class_name,
                "score": float(scores[i]),
                "box": [x1, y1, x2, y2],
            })
    return detections


def iou(box_a, box_b):
    """Compute Intersection over Union of two boxes."""
    xa1, ya1, xa2, ya2 = box_a
    xb1, yb1, xb2, yb2 = box_b

    inter_x1 = max(xa1, xb1)
    inter_y1 = max(ya1, yb1)
    inter_x2 = min(xa2, xb2)
    inter_y2 = min(ya2, yb2)

    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    area_a = (xa2 - xa1) * (ya2 - ya1)
    area_b = (xb2 - xb1) * (yb2 - yb1)
    union = area_a + area_b - inter_area
    return inter_area / union if union > 0 else 0.0


def voc_ap(recall, precision):
    """Compute Average Precision using the VOC method."""
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))
    for i in range(len(mpre) - 1, 0, -1):
        mpre[i - 1] = max(mpre[i - 1], mpre[i])
    indices = np.where(mrec[1:] != mrec[:-1])[0]
    ap = np.sum((mrec[indices + 1] - mrec[indices]) * mpre[indices + 1])
    return ap


def evaluate_detections(gt_list, pred_list, num_classes, iou_thresh=0.5):
    """Evaluate detections and return mAP and per-class AP."""
    aps = []
    for cls_id in range(1, num_classes + 1):
        cls_gts = [
            [ann for ann in gts if ann["category_id"] == cls_id]
            for gts in gt_list
        ]
        cls_preds = [
            sorted([
                det for det in preds if det["class_id"] == cls_id
            ], key=lambda d: d["score"], reverse=True)
            for preds in pred_list
        ]
        npos = sum(len(gts) for gts in cls_gts)
        if npos == 0:
            aps.append(0.0)
            continue
        matched = [set() for _ in cls_gts]
        image_preds = [
            (img_idx, det)
            for img_idx, preds in enumerate(cls_preds)
            for det in preds
        ]
        image_preds.sort(key=lambda x: x[1]["score"], reverse=True)
        tp = np.zeros(len(image_preds))
        fp = np.zeros(len(image_preds))
        for idx, (img_idx, pred) in enumerate(image_preds):
            gts = cls_gts[img_idx]
            ious = [
                iou(pred["box"], [gt["bbox"][0], gt["bbox"][1],
                                     gt["bbox"][0] + gt["bbox"][2],
                                     gt["bbox"][1] + gt["bbox"][3]])
                for gt in gts
            ]
            if ious and max(ious) >= iou_thresh:
                gt_idx = int(np.argmax(ious))
                if gt_idx not in matched[img_idx]:
                    tp[idx] = 1
                    matched[img_idx].add(gt_idx)
                else:
                    fp[idx] = 1
            else:
                fp[idx] = 1
        tp_cum = np.cumsum(tp)
        fp_cum = np.cumsum(fp)
        recall = tp_cum / npos
        precision = tp_cum / np.maximum(tp_cum + fp_cum, np.finfo(float).eps)
        aps.append(voc_ap(recall, precision))
    mAP = float(np.mean(aps)) if aps else 0.0
    return mAP, aps


def load_dataset(images_dir, annotation_file):
    """Load dataset annotations."""
    with open(annotation_file, "r", encoding="utf-8") as f:
        ann = json.load(f)
    images = {img["id"]: img["file_name"] for img in ann["images"]}
    annotations = {}
    for a in ann["annotations"]:
        annotations.setdefault(a["image_id"], []).append(a)
    categories = {c["id"]: c["name"] for c in ann["categories"]}
    ordered_images = [
        (img_id, images[img_id], annotations.get(img_id, []))
        for img_id in images
    ]
    return ordered_images, categories


def main():
    parser = argparse.ArgumentParser(description="Benchmark perception models")
    parser.add_argument("--images", required=True,
                        help="Path to image directory")
    parser.add_argument("--annotations", required=True,
                        help="Path to COCO annotations JSON")
    parser.add_argument("--model",
                        default="src/apm_core/models/ssd_mobilenet_v2.onnx",
                        help="Path to ONNX detection model")
    parser.add_argument("--output", default="benchmark_results.json",
                        help="File to write metrics")
    args = parser.parse_args()

    images, categories = load_dataset(args.images, args.annotations)
    session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    input_size = (320, 320)
    gt_list = []
    pred_list = []

    for _, file_name, anns in images:
        path = Path(args.images) / file_name
        img = cv2.imread(str(path))
        if img is None:
            continue
        inp = preprocess_image(img, input_size)
        outputs = session.run(None, {input_name: inp})
        preds = process_detections(outputs, img.shape, categories)
        gt_list.append(anns)
        pred_list.append(preds)

    mAP, ap_per_class = evaluate_detections(
        gt_list, pred_list, len(categories)
    )
    results = {
        "mAP": mAP,
        "AP_per_class": {
            categories[idx + 1]: ap_per_class[idx]
            for idx in range(len(ap_per_class))
        },
    }
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"Results written to {args.output}")


if __name__ == "__main__":
    main()
