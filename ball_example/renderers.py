import cv2
import math
import numpy as np
from typing import List, Optional, Tuple
from models import Ball, ArucoMarker, ArucoHitter


def draw_line(
    frame: np.ndarray,
    p1: Tuple[int, int],
    p2: Tuple[int, int],
    color: Tuple[int, int, int] = (255, 255, 255),
    thickness: int = 2
) -> None:
    """
    Draw a straight line on the frame between two points.
    Modifies the frame in place.

    Args:
        frame: the image to draw on
        p1: starting point (x1, y1)
        p2: ending point (x2, y2)
        color: BGR color tuple
        thickness: line thickness
    """
    cv2.line(frame, p1, p2, color, thickness)


def render_overlay(
    frame: np.ndarray,
    balls: List[Ball],
    markers: Optional[List[ArucoMarker]] = None,
    line_points: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None
) -> np.ndarray:
    """
    Draw detected balls with their direction arrows and speed labels,
    plus detected ArUco markers (corners, outline, center, and ID),
    then draw a scenario-specific line if provided.
    """
    annotated = frame.copy()

    # --- draw balls ---
    for ball in balls:
        x, y = ball.center
        r = ball.radius
        orig_color = ball.color or (0, 255, 0)
        # Contrast color for annotations
        color = tuple(255 - c for c in orig_color)

        # Ball boundary & center
        cv2.circle(annotated, (x, y), r, color, 2)
        cv2.circle(annotated, (x, y), 3, color, -1)

        # Velocity arrow & speed label
        vx, vy = map(float, ball.velocity)
        mag = math.hypot(vx, vy)
        if mag > 0:
            dx, dy = vx / mag, vy / mag
            length = max(2 * r, 30)
            end_pt = (int(x + dx * length), int(y + dy * length))
            cv2.arrowedLine(annotated, (x, y), end_pt, color, 2, tipLength=0.2)
            speed_text = f"{mag:.1f}px/s"
            cv2.putText(
                annotated, speed_text,
                (end_pt[0] + 5, end_pt[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )

    # --- draw ArUco markers ---
    if markers:
        for m in markers:
            pts = np.array(m.corners, dtype=np.int32).reshape(-1, 1, 2)
            outline_color = (0, 255, 255)
            corner_color  = (255, 0, 255)
            center_color  = (0, 128, 255)
            cx, cy = m.center

            if not isinstance(m, ArucoHitter):
                cv2.polylines(annotated, [pts], isClosed=True, color=outline_color, thickness=2)
                for idx, (px, py) in enumerate(m.corners):
                    cv2.circle(annotated, (px, py), 5, corner_color, -1)
                    cv2.putText(
                        annotated, str(idx),
                        (px + 5, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, corner_color, 1
                    )
                cv2.putText(
                    annotated, f"ID:{m.id}",
                    (cx + 5, cy + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, center_color, 2
                )

            # Always draw center point
            cv2.circle(annotated, (cx, cy), 5, center_color, -1)

    # --- draw scenario-specific line ---
    if line_points:
        pt1, pt2 = line_points
        draw_line(annotated, pt1, pt2)

    return annotated
