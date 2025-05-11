import cv2
import math
import numpy as np
from typing import List, Optional
from models import Ball, ArucoMarker

def render_overlay(
    frame: np.ndarray,
    balls: List[Ball],
    markers: Optional[List[ArucoMarker]] = None
) -> np.ndarray:
    """
    Draw detected balls with their direction arrows and speed labels,
    plus detected ArUco markers (corners, outline, center, and ID).
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
            # outline polygon
            pts = np.array(m.corners, dtype=np.int32).reshape(-1, 1, 2)
            outline_color = (0, 255, 255)      # yellow
            corner_color  = (255, 0, 255)      # magenta
            center_color  = (0, 128, 255)      # orange

            cv2.polylines(annotated, [pts], isClosed=True, color=outline_color, thickness=2)

            # corners (with index)
            for idx, (cx, cy) in enumerate(m.corners):
                cv2.circle(annotated, (cx, cy), 5, corner_color, -1)
                cv2.putText(
                    annotated, str(idx),
                    (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, corner_color, 1
                )

            # center point and ID label
            cx, cy = m.center
            cv2.circle(annotated, (cx, cy), 5, center_color, -1)
            cv2.putText(
                annotated, f"ID:{m.id}",
                (cx + 5, cy + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, center_color, 2
            )

    return annotated