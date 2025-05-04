import cv2
import math
import numpy as np
from typing import List
from models import Ball


def render_overlay(frame: np.ndarray, balls: List[Ball]) -> np.ndarray:
    """
    Draw detected balls with their direction arrows and speed labels.
    Arrow direction based on velocity vector, scaled for visibility.
    """
    annotated = frame.copy()

    for ball in balls:
        x, y = ball.center
        r = ball.radius
        orig_color = ball.color or (0, 255, 0)
        # Contrast color for annotations
        color = tuple(255 - c for c in orig_color)

        # Draw the ball's boundary and center
        cv2.circle(annotated, (x, y), r, color, 2)
        cv2.circle(annotated, (x, y), 3, color, -1)

        # Draw normalized direction arrow
        vx, vy = ball.velocity
        # Ensure Python floats, not numpy types
        vx, vy = float(vx), float(vy)
        mag = math.hypot(vx, vy)
        if mag > 0:
            dx, dy = vx / mag, vy / mag
            # Set arrow length: at least twice radius, or 30 px
            length = max(2 * r, 30)
            end_pt = (int(x + dx * length), int(y + dy * length))
            cv2.arrowedLine(
                annotated,
                (x, y),
                end_pt,
                color,
                2,
                tipLength=0.2
            )
            # Label speed
            speed_text = f"{mag:.1f}px/s"
            text_pos = (end_pt[0] + 5, end_pt[1] - 5)
            cv2.putText(
                annotated,
                speed_text,
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1
            )

    return annotated