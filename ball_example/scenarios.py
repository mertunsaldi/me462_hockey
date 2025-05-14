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
    """
    • First, calibrate the attached PlotClock (internal FSM in PlotClock).
    • Then draw ball→target line and label the start point in mm.
    """
    def __init__(self, plotclock: PlotClock):
        self.clock = plotclock
        # dynamic objects
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball:   Optional[Ball] = None
        # drawing state
        self._line: Optional[Tuple[Tuple[int,int],Tuple[int,int]]] = None
        self._start_px: Optional[Tuple[int,int]] = None
        self._start_mm: Optional[Tuple[float,float]] = None

    # ---------------------------------------------------------
    def update(self, detections):
        # 1) ensure calibration complete
        if not self.clock.calibration:
            self.clock.calibrate(detections)
            # show nothing until calibrated
            return

        # 2) normal gameplay logic
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
            dx,dy = tx-bx, ty-by
            dist = np.hypot(dx,dy)
            if dist>0:
                sx = int(bx - dx/dist*50)
                sy = int(by - dy/dist*50)
                self._start_px = (sx,sy)
                # mm conversion
                self._start_mm = self.clock.find_mm(sx,sy)

    # --------------------------------------------------------- drawing helpers
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
