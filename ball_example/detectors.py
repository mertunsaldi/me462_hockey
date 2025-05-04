import cv2
import numpy as np
from typing import List
from models import Ball

# Global background subtractor for motion detection
_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=500, varThreshold=50, detectShadows=False
)


def detect_balls(
    frame: np.ndarray,
    dp: float = 1.2,
    min_dist: float = 100,
    param1: float = 100,
    param2: float = 100,
    min_radius: int = 40,
    max_radius: int = 200
) -> List[Ball]:
    """
    Multi-strategy ball detection:
      1. Static circles via HoughCircles
      2. Dynamic blobs via background subtraction + contour circularity
      3. Blob detection fallback
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    balls: List[Ball] = []

    # 1. Static detection with HoughCircles
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp, min_dist,
        param1=param1, param2=param2,
        minRadius=min_radius, maxRadius=max_radius
    )
    if circles is not None:
        for x, y, r in np.round(circles[0]).astype(int):
            if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                color = tuple(int(c) for c in frame[y, x])
            else:
                color = (0, 255, 0)
            balls.append(Ball(center=(x, y), radius=r, color=color))

    # 2. Dynamic detection with background subtraction
    mask = _bg_subtractor.apply(frame)
    _, thresh = cv2.threshold(mask, 244, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < np.pi * (min_radius ** 2) or area > np.pi * (max_radius ** 2):
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim ** 2)
        if circ < 0.75:
            continue
        (x_f, y_f), r_f = cv2.minEnclosingCircle(cnt)
        x, y, r = int(x_f), int(y_f), int(r_f)
        if any(np.hypot(x - b.center[0], y - b.center[1]) < max(r, b.radius) * 0.6 for b in balls):
            continue
        color = tuple(int(c) for c in frame[y, x]) if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0] else (0, 255, 0)
        balls.append(Ball(center=(x, y), radius=r, color=color))

    # 3. Fallback: blob detection if still no balls
    if not balls:
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = np.pi * (min_radius ** 2)
        params.maxArea = np.pi * (max_radius ** 2)
        params.filterByCircularity = True
        params.minCircularity = 0.7
        params.filterByConvexity = True
        params.minConvexity = 0.8
        params.filterByInertia = True
        params.minInertiaRatio = 0.5
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(blurred)
        for kp in keypoints:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            r = int(kp.size / 2)
            color = tuple(int(c) for c in frame[y, x]) if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0] else (0, 255, 0)
            balls.append(Ball(center=(x, y), radius=r, color=color))

    return balls


def estimate_velocities(
    prev_balls: List[Ball],
    curr_balls: List[Ball],
    alpha: float = 0.5,
    raw_noise_thresh: float = 1.0,
    noise_thresh: float = 0.5
) -> None:
    """
    Compute and smooth velocities per ball:
      - Zero out raw displacement below raw_noise_thresh
      - Apply exponential moving average
      - Zero out smoothed velocities below noise_thresh
      - Round to one decimal place
      Matches by Ball.id so all balls remain annotated.
    """
    prev_map = {b.id: b for b in prev_balls}
    for curr in curr_balls:
        prev = prev_map.get(curr.id)
        if prev is not None:
            raw_vx = float(curr.center[0] - prev.center[0])
            raw_vy = float(curr.center[1] - prev.center[1])
            # Threshold out small raw jitter
            if np.hypot(raw_vx, raw_vy) < raw_noise_thresh:
                raw_vx, raw_vy = 0.0, 0.0
            old_vx, old_vy = prev.velocity
            vx = alpha * raw_vx + (1 - alpha) * old_vx
            vy = alpha * raw_vy + (1 - alpha) * old_vy
            # Threshold smoothed noise
            if np.hypot(vx, vy) < noise_thresh:
                vx, vy = 0.0, 0.0
            curr.velocity = (round(vx, 1), round(vy, 1))
        else:
            curr.velocity = (0.0, 0.0)
