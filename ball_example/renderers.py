import cv2
import math
import numpy as np
from typing import List, Optional, Tuple
from .models import Ball, ArucoMarker, ArucoHitter


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


def draw_points(
    frame: np.ndarray,
    points: List[Tuple[int, int]],
    color: Tuple[int, int, int] = (0, 0, 255),
    radius: int = 5,
    labels: Optional[List[str]] = None
) -> None:
    """
    Draw filled circles at given points and optional labels.

    Args:
        frame: the image to draw on
        points: list of (x, y) tuples
        color: BGR color tuple for points and text
        radius: radius of each point circle
        labels: optional list of text labels for each point
    """
    for idx, (x, y) in enumerate(points):
        cv2.circle(frame, (x, y), radius, color, -1)
        if labels and idx < len(labels):
            cv2.putText(
                frame,
                labels[idx],
                (x + radius + 2, y + radius + 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1
            )


def render_overlay(
    frame: np.ndarray,
    balls: List[Ball],
    markers: Optional[List[ArucoMarker]] = None,
    line_points: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None,
    extra_points: Optional[List[Tuple[int, int]]] = None,
    extra_labels: Optional[List[str]] = None,
    highlight: Optional[dict] = None,
) -> np.ndarray:
    """
    Draw detected balls, ArUco markers, a scenario line, and extra points.

    Args:
        frame: input image
        balls: list of Ball objects
        markers: optional list of ArUcoMarker objects
        line_points: optional endpoints (p1, p2) to draw a line
        extra_points: optional list of (x, y) to draw additional markers
        extra_labels: optional list of labels corresponding to extra_points

    Returns:
        annotated image copy
    """
    annotated = frame.copy()

    # --- draw balls ---
    for ball in balls:
        x, y = ball.center
        r = ball.radius
        orig_color = ball.color or (0, 255, 0)
        color = tuple(255 - c for c in orig_color)
        if highlight and highlight.get("type") == "ball" and ball.id == highlight.get("id"):
            color = (0, 0, 255)
        cv2.circle(annotated, (x, y), r, color, 2)
        cv2.circle(annotated, (x, y), 3, color, -1)
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
            if highlight and highlight.get("type") == "obs" and str(m.id) == str(highlight.get("id")):
                outline_color = (0, 0, 255)
            cx, cy = m.center
            if not isinstance(m, ArucoHitter):
                cv2.polylines(annotated, [pts], True, outline_color, 2)
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
            cv2.circle(annotated, (cx, cy), 5, center_color, -1)

    # --- draw line ---
    if line_points:
        pt1, pt2 = line_points
        draw_line(annotated, pt1, pt2)

    # --- draw extra points ---
    if extra_points:
        draw_points(annotated, extra_points, labels=extra_labels)

    return annotated
