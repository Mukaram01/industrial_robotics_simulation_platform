"""Example script to profile a dummy perception pipeline."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from profiling_utils import run_with_cprofile, run_with_pyinstrument


def perception_pipeline(images: int) -> None:
    """Run a simple image processing loop."""
    for _ in range(images):
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _ = cv2.Canny(gray, 100, 200)
        time.sleep(0.001)


def main() -> None:
    parser = argparse.ArgumentParser(description="Profile the perception pipeline")
    parser.add_argument("--images", type=int, default=100, help="Number of images to process")
    parser.add_argument("--profiler", choices=["cprofile", "pyinstrument"], default="cprofile")
    parser.add_argument("--output", default="perception_profile.prof", help="Output file")
    args = parser.parse_args()

    if args.profiler == "pyinstrument":
        run_with_pyinstrument(perception_pipeline, args.images, output=args.output)
    else:
        run_with_cprofile(perception_pipeline, args.images, output=args.output)
    print(f"Profiling complete. Results written to {args.output}")


if __name__ == "__main__":
    main()
