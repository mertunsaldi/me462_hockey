import numpy as np
from typing import List, Optional, Tuple, Union
from models import Ball, ArucoMarker, ArucoHitter

class Scenario:
    """
    Base class for game scenarios.
    Subclasses should implement update(), get_line_points(),
    get_extra_points(), and get_extra_labels().
    """
    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        raise NotImplementedError("Subclasses must implement update()")

    def get_line_points(self) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        return None

    def get_extra_points(self) -> Optional[List[Tuple[int, int]]]:
        return None

    def get_extra_labels(self) -> Optional[List[str]]:
        return None


class StandingBallHitter(Scenario):
    """
    Scenario where a stationary hitter (ArucoHitter) strikes a Ball towards a target (ArucoMarker).
    Computes the line between the ball and the target,
    and marks an initial hitter position 50px behind the ball.
    """
    def __init__(self):
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball: Optional[Ball] = None
        self.line_points: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None
        self.initial_hitter_pos: Optional[Tuple[int, int]] = None

    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        # Reset state
        self.hitter = None
        self.target = None
        self.ball = None
        self.line_points = None
        self.initial_hitter_pos = None

        # Classify detections
        for obj in detections:
            if isinstance(obj, ArucoHitter):
                self.hitter = obj
            elif isinstance(obj, ArucoMarker):
                self.target = obj
            elif isinstance(obj, Ball):
                self.ball = obj

        # Compute line and initial hitter position if both ball and target present
        if self.ball and self.target:
            x1, y1 = self.ball.center
            x2, y2 = self.target.center
            self.line_points = ((x1, y1), (x2, y2))

            dx = x2 - x1
            dy = y2 - y1
            # Compute unit direction from ball to target
            dist = np.hypot(dx, dy)
            backward_gap = 200  # px
            if dist > 0:
                ux = dx / dist
                uy = dy / dist
                # Point 50px behind the ball (opposite direction)
                ix = int(x1 - ux * backward_gap)
                iy = int(y1 - uy * backward_gap)
                self.initial_hitter_pos = (ix, iy)

    def get_line_points(self) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        return self.line_points

    def get_extra_points(self) -> Optional[List[Tuple[int, int]]]:
        if self.initial_hitter_pos:
            return [self.initial_hitter_pos]
        return None

    def get_extra_labels(self) -> Optional[List[str]]:
        if self.initial_hitter_pos:
            return ["Start"]
        return None
