import cv2
import numpy as np
from typing import List
from models import Ball, ArucoMarker

# Global background subtractor for motion detection
_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=500, varThreshold=50, detectShadows=False
)


class ArucoDetector:
    """Detect ArUco markers and return their corners + center."""
    # use the new API:
    _DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    _PARAMS = cv2.aruco.DetectorParameters()

    @staticmethod
    def detect(frame: np.ndarray) -> List[ArucoMarker]:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, ArucoDetector._DICT, parameters=ArucoDetector._PARAMS
        )

        markers: List[ArucoMarker] = []
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape(-1, 2)
                pts_int = [(int(x), int(y)) for x, y in pts]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                markers.append(
                    ArucoMarker(id=int(marker_id), corners=pts_int, center=(cx, cy))
                )

        return markers


class BallDetector:
    # ───── Tunable thresholds ───────────────────────────
    CIRCULARITY_THRESHOLD = 0.8
    AREA_RATIO_THRESHOLD   = 0.80

    # HSV range for your ball color (tweak these!)
    HSV_LOWER = np.array([  5, 100, 100], dtype=np.uint8)
    HSV_UPPER = np.array([ 25, 255, 255], dtype=np.uint8)
    # ────────────────────────────────────────────────────

    @staticmethod
    def detect(
        frame: np.ndarray,
        dp: float       = 1.2,
        min_dist: float = 100,
        param1: float   = 100,
        param2: float   = 100,
        min_radius: int = 40,
        max_radius: int = 200
    ) -> List[Ball]:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        balls: List[Ball] = []

        # 1) Static Hough (unchanged) …
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp, min_dist,
            param1=param1, param2=param2,
            minRadius=min_radius, maxRadius=max_radius
        )
        if circles is not None:
            for x, y, r in np.round(circles[0]).astype(int):
                color = tuple(int(c) for c in frame[y, x]) if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0] else (0,255,0)
                balls.append(Ball(center=(x,y), radius=r, color=color))

        # 2) Build combined mask: BG-subtractor + HSV color mask
        bg_mask = _bg_subtractor.apply(frame)
        _, fg_mask = cv2.threshold(bg_mask, 244, 255, cv2.THRESH_BINARY)

        hsv        = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, BallDetector.HSV_LOWER, BallDetector.HSV_UPPER)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        fg_clean    = cv2.morphologyEx(fg_mask,    cv2.MORPH_OPEN,  kernel, iterations=2)
        fg_clean    = cv2.morphologyEx(fg_clean,   cv2.MORPH_CLOSE, kernel, iterations=2)
        color_clean = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        combined = cv2.bitwise_and(fg_clean, color_clean)
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < np.pi*(min_radius**2) or area > np.pi*(max_radius**2):
                continue

            perim = cv2.arcLength(cnt, True)
            if perim == 0:
                continue

            circ = 4*np.pi*area/(perim**2)
            if circ < BallDetector.CIRCULARITY_THRESHOLD:
                continue

            (x_f, y_f), r_f = cv2.minEnclosingCircle(cnt)
            area_ratio = area / (np.pi*(r_f**2))
            if area_ratio < BallDetector.AREA_RATIO_THRESHOLD:
                continue

            x, y, r = int(x_f), int(y_f), int(r_f)
            if any(np.hypot(x-b.center[0], y-b.center[1]) < max(r, b.radius)*0.6 for b in balls):
                continue

            color = tuple(int(c) for c in frame[y, x]) if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0] else (0,255,0)
            balls.append(Ball(center=(x,y), radius=r, color=color))

        # 3) Fallback blob detector …
        if not balls:
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea       = True
            params.minArea            = np.pi*(min_radius**2)
            params.maxArea            = np.pi*(max_radius**2)
            params.filterByCircularity= True
            params.minCircularity     = BallDetector.CIRCULARITY_THRESHOLD
            params.filterByConvexity  = True
            params.minConvexity       = 0.8
            params.filterByInertia    = True
            params.minInertiaRatio    = 0.5

            detector  = cv2.SimpleBlobDetector_create(params)
            keypoints = detector.detect(blurred)
            for kp in keypoints:
                x, y = int(kp.pt[0]), int(kp.pt[1])
                r    = int(kp.size/2)
                color= tuple(int(c) for c in frame[y, x]) if 0<=x<frame.shape[1] and 0<=y<frame.shape[0] else (0,255,0)
                balls.append(Ball(center=(x,y), radius=r, color=color))

        return balls