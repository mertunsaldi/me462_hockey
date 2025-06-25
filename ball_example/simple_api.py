"""Simple command based scenario interpreter.

Clients supply a list of action dictionaries which are executed in order.
Supported actions:

* ``{"action": "calibrate", "clock": 0}``
* ``{"action": "hit_to", "clock": 1, "x": 10.0, "y": 20.0}``
* ``{"action": "defend", "clock": 2}``

The heavy lifting (striking/defending) is handled by built-in scenario
classes.
"""

from typing import List, Dict, Any, Iterable
import math

from .scenarios import Scenario, FixedTargetAttacker, BallReflector
from .gadgets import PlotClock


def sort_plotclocks(clocks: Iterable["PlotClock"]) -> List[PlotClock]:
    """Sort clocks and assign IDs based on distance and angle.

    The closest clock to the global origin becomes ID 0. Remaining
    clocks receive increasing IDs in clockwise order.
    """
    tmp = []
    for c in clocks:
        if not c.calibration:
            raise ValueError("All PlotClocks must be calibrated")
        ox, oy = c.calibration["origin_px"]
        r = (ox ** 2 + oy ** 2) ** 0.5
        angle = math.atan2(oy, ox) % (2 * math.pi)
        tmp.append((c, r, angle))

    if not tmp:
        return []

    tmp.sort(key=lambda t: t[1])  # by distance
    nearest = tmp.pop(0)
    others = tmp
    others.sort(key=lambda t: -t[2])  # clockwise by angle
    ordered = [nearest[0]] + [t[0] for t in others]

    # assign ids according to order
    for idx, c in enumerate(ordered):
        setattr(c, "id", idx)

    return ordered


class CommandScenario(Scenario):
    def __init__(self, clocks: List[PlotClock], frame_size, commands: List[Dict[str, Any]]):
        super().__init__()
        self.clocks = clocks
        self.frame_size = frame_size
        self.commands = list(commands)
        self._index = 0
        self._active: Scenario | None = None

    def _next(self):
        if self._active:
            self._active.on_stop()
        self._active = None
        self._index += 1
        if self._index >= len(self.commands):
            self.finished = True

    def update(self, detections):
        if self.finished:
            if self._active:
                self._active.update(detections)
            return

        if self._index >= len(self.commands):
            self.finished = True
            return

        cmd = self.commands[self._index]
        action = cmd.get("action")
        clock_id = int(cmd.get("clock", 0))
        if clock_id < 0 or clock_id >= len(self.clocks):
            raise ValueError(f"Unknown clock id {clock_id}")
        clock = self.clocks[clock_id]

        if action == "calibrate":
            if not clock.calibration:
                clock.calibrate(detections)
            else:
                self._next()
            return

        if self._active is None:
            if action == "hit_to":
                target = (float(cmd["x"]), float(cmd["y"]))
                self._active = FixedTargetAttacker(clock, self.frame_size, target)
                self._active.on_start()
            elif action == "defend":
                self._active = BallReflector(clock, self.frame_size)
                self._active.on_start()
            else:
                raise ValueError(f"Unknown action {action}")

        self._active.update(detections)

        if isinstance(self._active, FixedTargetAttacker):
            if self._active._phase is None and self._active._locked_M is None:
                self._next()
        # BallReflector runs indefinitely until externally stopped

    def process_message(self, message: Dict[str, Any]) -> None:
        if self._active:
            self._active.process_message(message)
