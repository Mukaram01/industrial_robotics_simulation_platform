#!/usr/bin/env python3
"""Launch an experiment from a configuration file."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


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


def load_config(path: Path) -> dict:
    """Load a YAML or JSON configuration file."""
    if path.suffix in {".yaml", ".yml"}:
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def build_command(opts: dict) -> list[str]:
    """Return the ROS launch command for the experiment."""
    scenario = opts.get("scenario", "default")
    realsense = "true" if opts.get("use_realsense") else "false"
    adv = "true" if opts.get("use_advanced_perception") else "false"
    return [
        "ros2",
        "launch",
        "simulation_tools",
        "integrated_system_launch.py",
        f"use_realsense:={realsense}",
        f"use_advanced_perception:={adv}",
        f"scenario:={scenario}",
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Run an experiment")
    parser.add_argument(
        "--config",
        required=True,
        help="Path to YAML or JSON configuration file",
    )
    args = parser.parse_args()

    ensure_ros2_humble()

    config = load_config(Path(args.config))
    cmd = build_command(config)
    subprocess.run(cmd, check=False)


if __name__ == "__main__":
    main()

