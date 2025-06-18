"""Utilities for running performance profilers."""

from __future__ import annotations

from typing import Any, Callable
import cProfile
import pstats


try:
    from pyinstrument import Profiler
except Exception:  # pyinstrument is optional
    Profiler = None  # type: ignore


__all__ = ["run_with_cprofile", "run_with_pyinstrument"]


def run_with_cprofile(func: Callable[..., Any], *args: Any, output: str = "profile.prof", **kwargs: Any) -> None:
    """Run ``func`` with ``cProfile`` and write stats to ``output``."""
    profiler = cProfile.Profile()
    profiler.enable()
    func(*args, **kwargs)
    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.dump_stats(output)


def run_with_pyinstrument(func: Callable[..., Any], *args: Any, output: str = "profile.txt", **kwargs: Any) -> None:
    """Run ``func`` with ``pyinstrument`` if available."""
    if Profiler is None:
        raise RuntimeError("pyinstrument is not installed")
    profiler = Profiler()
    profiler.start()
    func(*args, **kwargs)
    profiler.stop()
    with open(output, "w", encoding="utf-8") as f:
        f.write(profiler.output_text(unicode=True, color=False))
