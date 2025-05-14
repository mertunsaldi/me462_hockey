import numpy as np
import time
from typing import List, Optional, Tuple, Union, Dict, Any
from models import Ball, ArucoMarker, ArucoHitter
from gadgets import PlotClock


class Scenario:
    """Base class for game scenarios."""

    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        raise NotImplementedError

    def get_line_points(self):
        return None

    def get_extra_points(self):
        return None

    def get_extra_labels(self):
        return None


class StandingBallHitter(Scenario):
    """Calibrates PlotClock, then strikes the ball toward the target."""

    def __init__(self, plotclock: PlotClock):
        self.clock = plotclock
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball:   Optional[Ball] = None
        self._line:       Optional[Tuple[Tuple[int,int],Tuple[int,int]]] = None
        self._start_px:   Optional[Tuple[int,int]] = None
        self._start_mm:   Optional[Tuple[float,float]] = None
        self._fired = False          # ensure we send drawto sequence only once

    # ------------------------------------------------------------------
    def update(self, detections):
        # 1) drive calibration first
        if not self.clock.calibration:
            self.clock.calibrate(detections)
            return   # nothing else until calibrated

        # 2) reset per‑frame state
        self.hitter = self.target = self.ball = None
        self._line = self._start_px = self._start_mm = None

        for d in detections:
            if isinstance(d, ArucoHitter):   self.hitter = d
            elif isinstance(d, ArucoMarker): self.target  = d
            elif isinstance(d, Ball):        self.ball    = d

        if self.ball and self.target:
            bx,by = self.ball.center
            tx,ty = self.target.center
            self._line = ((bx,by),(tx,ty))

            # start point 60 px behind ball along target line
            dx,dy = tx-bx, ty-by
            dist = np.hypot(dx,dy)
            if dist>0:
                sx = int(bx - dx/dist*70)
                sy = int(by - dy/dist*70)
                self._start_px = (sx,sy)
                self._start_mm = self.clock.find_mm(sx,sy)

                # once per run: move hitter then strike
                if not self._fired and self._start_mm:
                    ball_mm = self.clock.find_mm(bx,by)
                    self.clock.send_command('drawto', *self._start_mm)
                    time.sleep(0.7)
                    self.clock.send_command('setxy', *ball_mm)
                    self._fired = True

    # ------------------------------------------------------------------
    def get_line_points(self):
        return self._line

    def get_extra_points(self):
        return [self._start_px] if self._start_px else None

    def get_extra_labels(self):
        if not self._start_px:
            return None
        if self._start_mm:
            return [f"Start ({self._start_mm[0]:.1f},{self._start_mm[1]:.1f} mm)"]
        return ["Start"]