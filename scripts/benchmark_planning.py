#!/usr/bin/env python3
"""Benchmark MoveIt planning for predefined poses."""

import argparse
import json
from time import perf_counter
from typing import List

import moveit_commander
import rclpy


def run_benchmark(group_name: str, poses: List[str], trials: int) -> List[dict]:
    """Execute planning for each pose and collect metrics."""
    moveit_commander.roscpp_initialize([])
    rclpy.init()

    group = moveit_commander.MoveGroupCommander(group_name)
    results = []

    for pose in poses:
        successes = 0
        planning_times = []
        traj_durations = []
        for _ in range(trials):
            group.set_named_target(pose)
            start = perf_counter()
            success, plan, _, _ = group.plan()
            planning_times.append(perf_counter() - start)
            if success:
                duration = 0.0
                try:
                    duration = (
                        plan.joint_trajectory.points[-1].time_from_start.to_sec()
                    )
                except Exception:
                    pass
                traj_durations.append(duration)
                group.execute(plan, wait=True)
                successes += 1
            group.clear_pose_targets()
        count = len(planning_times)
        results.append(
            {
                "pose": pose,
                "avg_planning_time": sum(planning_times) / count if count else 0.0,
                "success_rate": successes / count if count else 0.0,
                "avg_trajectory_duration": (
                    sum(traj_durations) / len(traj_durations)
                    if traj_durations
                    else 0.0
                ),
            }
        )

    rclpy.shutdown()
    moveit_commander.roscpp_shutdown()
    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark MoveIt planning for predefined poses"
    )
    parser.add_argument(
        "--group",
        default="manipulator",
        help="MoveIt planning group name",
    )
    parser.add_argument(
        "--poses",
        nargs="+",
        default=["home"],
        help="Named target poses to execute",
    )
    parser.add_argument(
        "--trials", type=int, default=5, help="Number of planning attempts"
    )
    parser.add_argument(
        "--output", default="planning_benchmark.json", help="Results file"
    )
    args = parser.parse_args()

    metrics = run_benchmark(args.group, args.poses, args.trials)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    print(f"Results written to {args.output}")


if __name__ == "__main__":
    main()

