import numpy as np
from typing import List, Optional, Tuple, Union
from models import Ball, ArucoMarker, ArucoHitter
from renderers import draw_line

class Scenario:
    """
    Base class for game scenarios.
    Subclasses should implement update() and optionally render() to draw scenario-specific overlays.
    """
    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        raise NotImplementedError("Subclasses must implement update()")

    def get_line_points(self) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        return None

    def render(self, frame: np.ndarray) -> None:
        """
        Draw any scenario-specific overlays onto the frame.
        By default, does nothing.
        """
        pass


class StandingBallHitter(Scenario):
    """
    Scenario where a stationary hitter (ArucoHitter) strikes a Ball towards a target (ArucoMarker).
    Draws the line between the ball and the target.
    """
    def __init__(self):
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball: Optional[Ball] = None
        self.line_points: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None

    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        # Reset state
        self.hitter = None
        self.target = None
        self.ball = None
        self.line_points = None

        # Classify detections
        for obj in detections:
            if isinstance(obj, ArucoHitter):
                self.hitter = obj
            elif isinstance(obj, ArucoMarker):
                self.target = obj
            elif isinstance(obj, Ball):
                self.ball = obj

        # Compute line if both ball and target present
        if self.ball and self.target:
            p1 = self.ball.center
            p2 = self.target.center
            self.line_points = (p1, p2)

    def get_line_points(self) -> Optional[Tuple[Tuple[int, int], Tuple[int, int]]]:
        return self.line_points

    def render(self, frame: np.ndarray) -> None:
        """
        Draw the connecting line between the ball and the target on the frame.
        """
        if self.line_points:
            pt1, pt2 = self.line_points
            draw_line(frame, pt1, pt2)
