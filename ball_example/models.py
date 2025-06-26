from typing import Tuple, Optional, List
import uuid


class ArucoMarker:
    """Simple container for a detected ArUco."""
    def __init__(self, id: int, corners: List[Tuple[int,int]], center: Tuple[int,int]):
        self.id = id
        self.corners = corners      # list of 4 (x,y) tuples in marker order
        self.center  = center       # (x,y) of the marker centroid


class ArucoHitter(ArucoMarker):
    """Simple container for a detected ArUco."""
    pass


class Obstacle(ArucoMarker):
    """Represents a detected obstacle marker."""
    pass


class ArucoManager(ArucoMarker):
    """Represents an arena manager ArUco marker."""
    pass


class Ball:
    """
    Represents a detected ball in the frame.

    Attributes:
        id: Unique identifier for tracking across frames.
        center: (x, y) pixel coordinates of the ball's center.
        radius: Radius of the ball in pixels.
        color: (B, G, R) color used for rendering (optional).
        velocity: (vx, vy) displacement per frame (initialized to zero).
    """
    def __init__(
        self,
        center: Tuple[int, int],
        radius: int,
        color: Optional[Tuple[int, int, int]] = None,
        ball_id: Optional[str] = None
    ):
        self.id: str = ball_id or str(uuid.uuid4())
        self.center: Tuple[int, int] = center
        self.radius: int = radius
        self.color: Optional[Tuple[int, int, int]] = color
        # Velocity in pixels per frame: (dx, dy)
        self.velocity: Tuple[float, float] = (0.0, 0.0)

    def __repr__(self) -> str:
        return (
            f"<Ball id={self.id[:8]} center={self.center} "
            f"radius={self.radius} velocity={self.velocity}>"
        )

