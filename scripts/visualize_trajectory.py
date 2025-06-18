#!/usr/bin/env python3
"""Launch RViz and related visualization nodes."""

import argparse
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize robot trajectory in RViz"
    )
    parser.add_argument(
        "--use-realsense",
        action="store_true",
        help="Use RealSense camera instead of the synthetic camera",
    )
    parser.add_argument(
        "--use-rviz",
        action="store_true",
        default=True,
        help="Launch RViz window",
    )
    args = parser.parse_args()

    cmd = [
        "ros2",
        "launch",
        "simulation_tools",
        "visualization_launch.py",
        f"use_realsense:={'true' if args.use_realsense else 'false'}",
        f"use_rviz:={'true' if args.use_rviz else 'false'}",
    ]
    subprocess.run(cmd, check=False)


if __name__ == "__main__":
    main()
