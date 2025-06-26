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
            if c.calibration:
                continue
            if c.device_id is not None:
                sub_dets = [
                    d
                    for d in detections
                    if not hasattr(d, "id") or d.id == c.device_id
                ]
            else:
                sub_dets = detections
            c.calibrate(sub_dets)
        if all(c.calibration for c in clocks):
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
