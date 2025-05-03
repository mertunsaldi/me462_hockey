import cv2
import numpy as np
from typing import List
from models import Ball


def render_overlay(frame: np.ndarray, balls: List[Ball]) -> np.ndarray:
    """
    Draw detected balls, their velocity arrows, and speed labels on the image with contrast.

    Args:
        frame: Original BGR image.
        balls: List of Ball objects with updated velocity.

    Returns:
        Annotated BGR image.
    """
    annotated = frame.copy()

    for ball in balls:
        x, y = ball.center
        r = ball.radius
        orig_color = ball.color or (0, 255, 0)
        # Compute contrasting color by inverting the sampled color
        color = tuple(255 - c for c in orig_color)

        # Draw circle boundary
        cv2.circle(annotated, (x, y), r, color, 2)
        # Draw center point
        cv2.circle(annotated, (x, y), 3, color, -1)

        # Draw velocity arrow
        vx, vy = ball.velocity
        end_point = (int(x + vx), int(y + vy))
        cv2.arrowedLine(
            annotated, (x, y), end_point,
            color, 2, tipLength=0.2
        )

        # Compute speed magnitude and render text
        speed = np.hypot(vx, vy)
        text = f"{speed:.1f}px/f"
        # Put text slightly offset from end of arrow
        text_pos = (end_point[0] + 5, end_point[1] - 5)
        cv2.putText(
            annotated, text, text_pos,
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
        )

    return annotated
