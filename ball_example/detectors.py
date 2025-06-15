import cv2
import numpy as np
from typing import List
from models import Ball, ArucoMarker, ArucoHitter

# Global background subtractor for motion detection
_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=500, varThreshold=50, detectShadows=False
)


class ArucoDetector:
    """Detect ArUco markers and return their corners + center."""
    # use the new API:
    _DICT6   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
    _DICT4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    _PARAMS = cv2.aruco.DetectorParameters()

    @staticmethod
    def detect(frame: np.ndarray) -> List[ArucoMarker]:
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners6, ids6, _ = cv2.aruco.detectMarkers(
            gray, ArucoDetector._DICT6, parameters=ArucoDetector._PARAMS
        )
        corners4, ids4, _ = cv2.aruco.detectMarkers(
            gray, ArucoDetector._DICT4, parameters=ArucoDetector._PARAMS
        )

        markers: List[ArucoMarker] = []
        if ids6 is not None:
            for marker_corners, marker_id in zip(corners6, ids6.flatten()):
                pts = marker_corners.reshape(-1, 2)
                pts_int = [(int(x), int(y)) for x, y in pts]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                markers.append(
                    ArucoMarker(id=int(marker_id), corners=pts_int, center=(cx, cy))
                )

        if ids4 is not None:
            for marker_corners, marker_id in zip(corners4, ids4.flatten()):
                pts = marker_corners.reshape(-1, 2)
                pts_int = [(int(x), int(y)) for x, y in pts]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                markers.append(
                    ArucoHitter(id=int(marker_id), corners=pts_int, center=(cx, cy))
                )

        return markers


class BallDetector:
    # ───── Tunable thresholds ───────────────────────────
    CIRCULARITY_THRESHOLD = 0.75
    AREA_RATIO_THRESHOLD   = 0.8

    # HSV range for your ball color (tweak these!)
    HSV_LOWER = np.array([0, 0, 0], dtype=np.uint8)
    HSV_UPPER = np.array([120, 120, 120], dtype=np.uint8)

    # ────────────────────────────────────────────────────

    @staticmethod
    def detect(
        frame: np.ndarray,
        dp: float       = 1.2,
        min_dist: float = 70,
        param1: float   = 70,
        param2: float   = 70,
        min_radius: int = 30,
        max_radius: int = 50,
        scale: float    = 1.0,
    ) -> List[Ball]:
        """Detect balls in ``frame``.

        ``scale`` allows the expensive processing to run on a smaller
        version of the image.  The returned ball coordinates are scaled
        back to the original resolution.
        """

        orig_frame = frame
        if scale != 1.0:
            frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Adjust thresholds for the scaled image
        min_dist_s   = int(min_dist * scale)
        min_radius_s = max(1, int(min_radius * scale))
        max_radius_s = int(max_radius * scale)

        balls: List[Ball] = []

        # 1) Static Hough (unchanged) …
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp, min_dist_s,
            param1=param1, param2=param2,
            minRadius=min_radius_s, maxRadius=max_radius_s
        )
        if circles is not None:
            for x, y, r in np.round(circles[0]).astype(int):
                if scale != 1.0:
                    x_o = int(x / scale)
                    y_o = int(y / scale)
                    r_o = int(r / scale)
                else:
                    x_o, y_o, r_o = x, y, r
                color = (
                    tuple(int(c) for c in orig_frame[y_o, x_o])
                    if 0 <= x_o < orig_frame.shape[1] and 0 <= y_o < orig_frame.shape[0]
                    else (0, 255, 0)
                )
                balls.append(Ball(center=(x_o, y_o), radius=r_o, color=color))

        # 2) Build combined mask: BG-subtractor + HSV color mask
        bg_mask = _bg_subtractor.apply(frame)
        _, fg_mask = cv2.threshold(bg_mask, 244, 255, cv2.THRESH_BINARY)

        hsv        = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, BallDetector.HSV_LOWER, BallDetector.HSV_UPPER)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        fg_clean    = cv2.morphologyEx(fg_mask,    cv2.MORPH_OPEN,  kernel, iterations=2)
        fg_clean    = cv2.morphologyEx(fg_clean,   cv2.MORPH_CLOSE, kernel, iterations=2)
        color_clean = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        #combined = cv2.bitwise_or(color_clean, fg_clean)
        combined = color_clean
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < np.pi * (min_radius_s ** 2) or area > np.pi * (max_radius_s ** 2):
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
            if scale != 1.0:
                x_o = int(x / scale)
                y_o = int(y / scale)
                r_o = int(r / scale)
            else:
                x_o, y_o, r_o = x, y, r
            if any(np.hypot(x_o - b.center[0], y_o - b.center[1]) < max(r_o, b.radius) * 0.6 for b in balls):
                continue

            color = (
                tuple(int(c) for c in orig_frame[y_o, x_o])
                if 0 <= x_o < orig_frame.shape[1] and 0 <= y_o < orig_frame.shape[0]
                else (0, 255, 0)
            )
            balls.append(Ball(center=(x_o, y_o), radius=r_o, color=color))
        # 3) Fallback blob detector …
        if not balls:
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea       = True
            params.minArea            = np.pi*(min_radius_s**2)
            params.maxArea            = np.pi*(max_radius_s**2)
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
                r    = int(kp.size / 2)
                if scale != 1.0:
                    x_o = int(x / scale)
                    y_o = int(y / scale)
                    r_o = int(r / scale)
                else:
                    x_o, y_o, r_o = x, y, r
                color = (
                    tuple(int(c) for c in orig_frame[y_o, x_o])
                    if 0 <= x_o < orig_frame.shape[1] and 0 <= y_o < orig_frame.shape[0]
                    else (0, 255, 0)
                )
                balls.append(Ball(center=(x_o, y_o), radius=r_o, color=color))

        return balls