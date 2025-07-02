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

class PhysicalTarget(ArucoMarker):
    """Represents a detected obstacle marker."""
    pass


class ArucoManager(ArucoMarker):
    """Represents an arena manager ArUco marker."""
    pass


class ArucoWall(ArucoMarker):
    """Represents a wall marker used to define the arena corners."""

    #: Default real size of one edge in millimetres
    DEFAULT_EDGE_MM: float = 24.0

    def __init__(
        self,
        id: int,
        corners: List[Tuple[int, int]],
        center: Tuple[int, int],
        *,
        real_edge_mm: float | None = None,
    ) -> None:
        super().__init__(id=id, corners=corners, center=center)
        self.real_edge_mm: float = (
            real_edge_mm if real_edge_mm is not None else ArucoWall.DEFAULT_EDGE_MM
        )


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


class Arena:
    """Represents the playing arena defined by ArucoWall markers."""

    def __init__(self, walls: List[ArucoWall]):
        self.walls = list(walls)

    def get_corner_positions(self) -> List[Tuple[int, int]]:
        """Return the center coordinates of all wall markers."""
        return [w.center for w in self.walls]

    def get_arena_corners(self) -> List[Tuple[int, int]]:
        """Return estimated corners of the arena.

        Long edges between markers are shifted toward the arena centre by
        half the marker edge length before calculating intersections.
        """
        import math

        if len(self.walls) < 2:
            return []

        cx = sum(w.center[0] for w in self.walls) / len(self.walls)
        cy = sum(w.center[1] for w in self.walls) / len(self.walls)

        def _angle(w: ArucoWall) -> float:
            return math.atan2(w.center[1] - cy, w.center[0] - cx)

        walls = sorted(self.walls, key=_angle)

        # Estimate pixel/mm scale from marker edge lengths
        px_per_mm_vals: list[float] = []
        for w in walls:
            p0, p1 = w.corners[0], w.corners[1]
            d = math.hypot(p0[0] - p1[0], p0[1] - p1[1])
            if d > 0:
                px_per_mm_vals.append(d / w.real_edge_mm)
        px_per_mm = sum(px_per_mm_vals) / len(px_per_mm_vals) if px_per_mm_vals else 1.0

        long_thresh = 3 * walls[0].real_edge_mm * px_per_mm
        offset = walls[0].real_edge_mm / 2 * px_per_mm

        def _shift_line(p1: tuple[float, float], p2: tuple[float, float]) -> tuple[tuple[float, float], tuple[float, float]]:
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            length = math.hypot(dx, dy)
            if length == 0:
                return p1, p2
            nx, ny = -dy / length, dx / length
            mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
            to_c = (cx - mx) * nx + (cy - my) * ny
            if to_c < 0:
                nx, ny = -nx, -ny
            sx, sy = nx * offset, ny * offset
            return (p1[0] + sx, p1[1] + sy), (p2[0] + sx, p2[1] + sy)

        def _intersect(a1, a2, b1, b2) -> tuple[float, float] | None:
            x1, y1 = a1
            x2, y2 = a2
            x3, y3 = b1
            x4, y4 = b2
            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(denom) < 1e-6:
                return None
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
            return px, py

        shifted_lines: list[tuple[tuple[float, float], tuple[float, float]]] = []
        edges: list[tuple[ArucoWall, ArucoWall, float]] = []
        for i, w in enumerate(walls):
            w2 = walls[(i + 1) % len(walls)]
            d = math.hypot(w.center[0] - w2.center[0], w.center[1] - w2.center[1])
            edges.append((w, w2, d))

        for w1, w2, dist in edges:
            if dist >= long_thresh:
                line = _shift_line(w1.center, w2.center)
                shifted_lines.append(line)

        if len(shifted_lines) < 2:
            return []

        corners: list[tuple[int, int]] = []
        for i in range(len(shifted_lines)):
            a1, a2 = shifted_lines[i]
            b1, b2 = shifted_lines[(i + 1) % len(shifted_lines)]
            pt = _intersect(a1, a2, b1, b2)
            if pt is not None:
                corners.append((int(round(pt[0])), int(round(pt[1]))))

        return corners


