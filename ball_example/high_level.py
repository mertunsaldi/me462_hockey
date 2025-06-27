"""High level helper utilities for PlotClock hockey.

This module provides convenience functions for discovering and
calibrating PlotClock objects from detection results as well as
starting common behaviors like attack or defense.
"""

import time
from typing import Iterable, List, Tuple, Callable, Any
from .models import (
    ArucoMarker,
    ArucoManager,
    ArucoWall,
    Arena, ArucoHitter,
)
from .gadgets import (
    PlotClock, ArenaManager

)
from .renderers import draw_line, draw_points
from .scenarios import FixedTargetAttacker, BallReflector


__all__ = [
    "discover_plotclocks",
    "calibrate_clocks",
    "draw_arena",
]


def discover_plotclocks(
    detections: Iterable[ArucoMarker], *, master: Any | None = None
) -> List[PlotClock]:
    """Return PlotClock objects based on ArUco detections.

    ``ArucoManager`` results produce :class:`ArenaManager` instances while
    ``ArucoHitter`` results produce regular :class:`PlotClock` objects.  Each
    device is assigned the detected marker ID and optional ``master`` for
    communication.  The clocks are not calibrated automatically.
    """
    clocks: List[PlotClock] = []
    for d in detections:
        if isinstance(d, ArucoManager):
            clocks.append(ArenaManager(device_id=d.id, master=master))
        elif isinstance(d, ArucoHitter):
            clocks.append(PlotClock(device_id=d.id, master=master))
    return clocks


def calibrate_clocks(
    clocks: Iterable[PlotClock],
    get_detections: Callable[[], Iterable],
    *,
    timeout: float = 10.0,
) -> None:
    """Calibrate each PlotClock in sequence using detection feedback.

    The previous implementation tried to calibrate all clocks concurrently,
    which resulted in overlapping serial commands when multiple devices were
    connected to the same master Pico.  This could cause responses for link
    length queries to be mixed with ``setXY`` commands.  The calibration order
    is now strictly sequential: each clock is fully calibrated before moving on
    to the next one.

    Parameters
    ----------
    clocks:
        Iterable of PlotClock objects.
    get_detections:
        Callable returning the latest detection list each iteration.
    timeout:
        Maximum time in seconds to attempt calibration **per clock**.
    """

    for clock in clocks:
        start = time.time()
        while time.time() - start < timeout:
            detections = list(get_detections())
            if not clock.calibration:
                clock.calibrate(detections)
            if clock.calibration:
                break
            time.sleep(0.05)


def draw_arena(frame, arena: Arena | List[ArucoWall]) -> None:
    """Draw an arena based on ArucoWall markers onto ``frame``."""

    if isinstance(arena, Arena):
        corners = arena.get_arena_corners()
    else:
        corners = Arena(list(arena)).get_arena_corners()

    if len(corners) < 2:
        return

    for i in range(len(corners)):
        p1 = corners[i]
        p2 = corners[(i + 1) % len(corners)]
        draw_line(frame, p1, p2)

    draw_points(frame, corners)
