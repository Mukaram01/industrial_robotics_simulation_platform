#!/usr/bin/env python3
"""Launch RViz and related visualization nodes."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


def ensure_ros2_humble() -> None:
    """Verify that the ROS 2 Humble environment is available."""
    if shutil.which("ros2") is None:
        sys.exit(
            "ros2 executable not found. Source /opt/ros/humble/setup.bash before running."
        )
    distro = os.environ.get("ROS_DISTRO")
    if distro and distro != "humble":
        print(
            f"Warning: running under ROS 2 '{distro}' (expected 'humble').",
            file=sys.stderr,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize robot trajectory in RViz",
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

    ensure_ros2_humble()

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

