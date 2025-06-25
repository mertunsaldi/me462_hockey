"""High level helper utilities for PlotClock hockey.

This module provides convenience functions for discovering and
calibrating PlotClock objects from detection results as well as
starting common behaviors like attack or defense.
"""

import time
from typing import Iterable, List, Tuple, Callable

from .gadgets import PlotClock, ArenaManager
from .models import (
    ArucoHitter,
    ArucoMarker,
    ArucoManager,
    ArucoWall,
    Arena,
)
from .renderers import draw_line, draw_points
from .scenarios import FixedTargetAttacker, BallReflector


__all__ = [
    "discover_plotclocks",
    "calibrate_clocks",
    "attack",
    "defend",
    "draw_arena",
]


def discover_plotclocks(detections: Iterable[ArucoMarker]) -> List[PlotClock]:
    """Return PlotClock objects based on ArUco detections.

    ``ArucoManager`` results produce :class:`ArenaManager` instances while
    ``ArucoHitter`` results produce regular :class:`PlotClock` objects.
    The clocks are not calibrated automatically.
    """
    clocks: List[PlotClock] = []
    for d in detections:
        if isinstance(d, ArucoManager):
            clocks.append(ArenaManager())
        elif isinstance(d, ArucoHitter):
            clocks.append(PlotClock())
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
            if not c.calibration:
                c.calibrate(detections)
        if all(c.calibration for c in clocks):
            break
        time.sleep(0.05)


def draw_arena(frame, arena: Arena | List[ArucoWall]) -> None:
    """Draw an arena based on ArucoWall markers onto ``frame``."""

    if isinstance(arena, Arena):
        walls = arena.walls
    else:
        walls = list(arena)

    points = [w.center for w in walls]

    # connect each point to its two nearest neighbors
    edges: set[tuple[int, int]] = set()
    for i, p in enumerate(points):
        dists = []
        for j, q in enumerate(points):
            if i == j:
                continue
            dx = p[0] - q[0]
            dy = p[1] - q[1]
            dists.append((dx * dx + dy * dy, j))
        dists.sort(key=lambda x: x[0])
        for _, j in dists[:2]:
            pair = tuple(sorted((i, j)))
            edges.add(pair)

    for i, j in edges:
        draw_line(frame, points[i], points[j])

    draw_points(frame, points)
