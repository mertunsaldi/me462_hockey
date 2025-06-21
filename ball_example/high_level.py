"""High level helper utilities for PlotClock hockey.

This module provides convenience functions for discovering and
calibrating PlotClock objects from detection results as well as
starting common behaviors like attack or defense.
"""

import time
from typing import Iterable, List, Tuple, Callable

from .gadgets import PlotClock
from .models import ArucoHitter, ArucoMarker
from .scenarios import FixedTargetAttacker, BallReflector


__all__ = [
    "discover_plotclocks",
    "calibrate_clocks",
    "attack",
    "defend",
]


def discover_plotclocks(detections: Iterable[ArucoMarker]) -> List[PlotClock]:
    """Return PlotClock objects based on ArUco hitter detections.

    Each detected :class:`ArucoHitter` spawns a new :class:`PlotClock`.
    The clocks are not calibrated automatically.
    """
    hitters = [d for d in detections if isinstance(d, ArucoHitter)]
    return [PlotClock() for _ in hitters]


def calibrate_clocks(
    clocks: Iterable[PlotClock],
    get_detections: Callable[[], Iterable],
    *,
    timeout: float = 10.0,
) -> None:
    """Repeatedly feed detections to ``PlotClock.calibrate`` until all clocks finish.

    Parameters
    ----------
    clocks:
        Iterable of PlotClock objects.
    get_detections:
        Callable returning the latest detection list each iteration.
    timeout:
        Maximum time in seconds to attempt calibration.
    """
    start = time.time()
    clocks = list(clocks)
    while time.time() - start < timeout:
        detections = list(get_detections())
        for c in clocks:
            if not c.calibration:
                c.calibrate(detections)
        if all(c.calibration for c in clocks):
            break
        time.sleep(0.05)


def attack(clock: PlotClock, frame_size: Tuple[int, int], target_mm: Tuple[float, float]):
    """Create a FixedTargetAttacker scenario for *clock*."""
    return FixedTargetAttacker(clock, frame_size, target_mm)


def defend(clock: PlotClock, frame_size: Tuple[int, int]):
    """Create a BallReflector scenario for *clock*."""
    return BallReflector(clock, frame_size)


