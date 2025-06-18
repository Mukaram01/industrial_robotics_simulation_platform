"""Example script to profile a simple simulation loop."""

from __future__ import annotations

import argparse
import time

from profiling_utils import run_with_cprofile, run_with_pyinstrument


def simulation_loop(steps: int) -> None:
    """Dummy simulation loop used for profiling."""
    value = 0
    for i in range(steps):
        value += i * i
        time.sleep(0.001)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Profile the simulation loop")
    parser.add_argument("--steps", type=int, default=1000, help="Number of loop iterations")
    parser.add_argument("--profiler", choices=["cprofile", "pyinstrument"], default="cprofile")
    parser.add_argument("--output", default="simulation_profile.prof", help="Output file")
    args = parser.parse_args()

    if args.profiler == "pyinstrument":
        run_with_pyinstrument(simulation_loop, args.steps, output=args.output)
    else:
        run_with_cprofile(simulation_loop, args.steps, output=args.output)
    print(f"Profiling complete. Results written to {args.output}")


if __name__ == "__main__":
    main()
