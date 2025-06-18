#!/usr/bin/env python3
"""Utility to launch the integrated simulation."""

import argparse
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Launch the full industrial simulation"
    )
    parser.add_argument(
        "--use-realsense",
        action="store_true",
        help="Use RealSense camera instead of the synthetic camera",
    )
    parser.add_argument(
        "--use-advanced-perception",
        action="store_true",
        help="Launch advanced perception nodes",
    )
    parser.add_argument(
        "--scenario",
        default="default",
        help="Scenario configuration to load",
    )
    args = parser.parse_args()

    cmd = [
        "ros2",
        "launch",
        "simulation_tools",
        "integrated_system_launch.py",
        f"use_realsense:={'true' if args.use_realsense else 'false'}",
        f"use_advanced_perception:={'true' if args.use_advanced_perception else 'false'}",
        f"scenario:={args.scenario}",
    ]
    subprocess.run(cmd, check=False)


if __name__ == "__main__":
    main()
