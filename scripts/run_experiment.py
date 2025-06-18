#!/usr/bin/env python3
"""Launch an experiment from a configuration file."""

import argparse
import json
from pathlib import Path
import subprocess

import yaml


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

    config = load_config(Path(args.config))
    cmd = build_command(config)
    subprocess.run(cmd, check=False)


if __name__ == "__main__":
    main()
