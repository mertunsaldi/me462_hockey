import cv2
import numpy as np
from typing import List, Optional
from .models import (
    Ball,
    ArucoMarker,
    ArucoHitter,
    Obstacle,
    ArucoManager,
    ArucoWall,
)

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
                if int(marker_id) in (0, 1):
                    markers.append(
                        Obstacle(id=int(marker_id), corners=pts_int, center=(cx, cy))
                    )
                else:
                    markers.append(
                        ArucoMarker(id=int(marker_id), corners=pts_int, center=(cx, cy))
                    )

        if ids4 is not None:
            for marker_corners, marker_id in zip(corners4, ids4.flatten()):
                if int(marker_id) in (17, 37):
                    continue  # ignore misdetected IDs
                pts = marker_corners.reshape(-1, 2)
                pts_int = [(int(x), int(y)) for x, y in pts]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                if int(marker_id) == 0:
                    markers.append(
                        ArucoManager(id=int(marker_id), corners=pts_int, center=(cx, cy))
                    )
                elif int(marker_id) == 13:
                    markers.append(
                        ArucoWall(id=int(marker_id), corners=pts_int, center=(cx, cy))
                    )
                else:
                    markers.append(
                        ArucoHitter(id=int(marker_id), corners=pts_int, center=(cx, cy))
                    )

        return markers


class BallDetector:
    # ───── Tunable thresholds ───────────────────────────
    CIRCULARITY_THRESHOLD = 0.9
    AREA_RATIO_THRESHOLD  = 0.8
    # reject contours with large holes
    SOLIDITY_THRESHOLD    = 0.9
    # edge density limit for Hough circles
    EDGE_DENSITY_THRESHOLD = 0.15

    # Gaussian blur parameters
    BLUR_KERNEL = 9  # odd integer
    BLUR_SIGMA  = 2.0

    # HSV range for your ball color (tweak these!)
    HSV_LOWER = np.array([0, 0, 0], dtype=np.uint8)
    HSV_UPPER = np.array([120, 120, 120], dtype=np.uint8)

    # Default ball size limits
    MIN_RADIUS = 17
    MAX_RADIUS = 50

    # ────────────────────────────────────────────────────

    @staticmethod
    def detect(
        frame: np.ndarray,
        *,
        mask: Optional[np.ndarray] = None,
        dp: float       = 1.2,
        min_dist: float = 70,
        param1: float   = 70,
        param2: float   = 70,
        min_radius: int | None = None,
        max_radius: int | None = None,
        scale: float    = 0.5,
    ) -> List[Ball]:
        """Detect balls in ``frame``.

        ``scale`` allows the expensive processing to run on a smaller
        version of the image.  The returned ball coordinates are scaled
        back to the original resolution.
        """

        if min_radius is None:
            min_radius = BallDetector.MIN_RADIUS
        if max_radius is None:
            max_radius = BallDetector.MAX_RADIUS

        orig_frame = frame
        if scale != 1.0:
            frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
            if mask is not None:
                mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        k = max(1, int(BallDetector.BLUR_KERNEL))
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(
            gray,
            (k, k),
            BallDetector.BLUR_SIGMA,
        )

        edges   = cv2.Canny(blurred, 100, 200)
        
        # Adjust thresholds for the scaled image
        min_dist_s   = int(min_dist * scale)
        min_radius_s = max(1, int(min_radius * scale))
        max_radius_s = int(max_radius * scale)

        balls: List[Ball] = []

        # 1) Static Hough (unchanged) …
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp, min_dist_s,
            param1=param1,
            param2=param2,
            minRadius=min_radius_s,
            maxRadius=max_radius_s,
        )
        if circles is not None:
            for x, y, r in np.round(circles[0]).astype(int):

                # check interior edge density to avoid hollow shapes
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), max(1, int(r * 0.8)), 255, -1)
                inner_edges = cv2.bitwise_and(edges, edges, mask=mask)
                density = np.count_nonzero(inner_edges) / (np.pi * (max(1, r * 0.8) ** 2))
                if density > BallDetector.EDGE_DENSITY_THRESHOLD:
                    continue

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

        # 2) Build or use provided mask
        if mask is None:
            bg_mask = _bg_subtractor.apply(frame)
            _, fg_mask = cv2.threshold(bg_mask, 244, 255, cv2.THRESH_BINARY)

            hsv        = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            color_mask = cv2.inRange(hsv, BallDetector.HSV_LOWER, BallDetector.HSV_UPPER)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            fg_clean    = cv2.morphologyEx(fg_mask,    cv2.MORPH_OPEN,  kernel, iterations=2)
            fg_clean    = cv2.morphologyEx(fg_clean,   cv2.MORPH_CLOSE, kernel, iterations=2)
            color_clean = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

            combined = color_clean
        else:
            combined = mask
        contours, hierarchy = cv2.findContours(combined, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        for idx, cnt in enumerate(contours):
            if hierarchy is not None and hierarchy[0][idx][3] != -1:
                continue  # skip holes
            hole_area = 0.0
            child = hierarchy[0][idx][2] if hierarchy is not None else -1
            while child != -1:
                hole_area += cv2.contourArea(contours[child])
                child = hierarchy[0][child][0]

            area = cv2.contourArea(cnt) - hole_area
            if area < np.pi * (min_radius_s ** 2) or area > np.pi * (max_radius_s ** 2):
                continue

            perim = cv2.arcLength(cnt, True)
            if perim == 0:
                continue

            circ = 4*np.pi*area/(perim**2)
            if circ < BallDetector.CIRCULARITY_THRESHOLD:
                continue

            (x_f, y_f), r_f = cv2.minEnclosingCircle(cnt)
            area_ratio = area / (np.pi * (r_f ** 2))
            solidity = area / cv2.contourArea(cv2.convexHull(cnt)) if cv2.contourArea(cv2.convexHull(cnt)) > 0 else 0
            if solidity < BallDetector.SOLIDITY_THRESHOLD:
                continue

            if area_ratio < BallDetector.AREA_RATIO_THRESHOLD:
                continue

            x, y, r = int(x_f), int(y_f), int(r_f)

            if any(np.hypot(int(x / scale if scale != 1.0 else x) - b.center[0], int(y / scale if scale != 1.0 else y) - b.center[1]) < max(int(r / scale if scale != 1.0 else r), b.radius) * 0.6 for b in balls):
                continue

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
        # 3) Fallback blob detector …
        if not balls:
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea       = True
            params.minArea            = np.pi*(min_radius_s**2)
            params.maxArea            = np.pi*(max_radius_s**2)
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
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), max(1, int(r * 0.8)), 255, -1)
                inner_edges = cv2.bitwise_and(edges, edges, mask=mask)
                density = np.count_nonzero(inner_edges) / (np.pi * (max(1, r * 0.8) ** 2))
                if density > BallDetector.EDGE_DENSITY_THRESHOLD:
                    continue

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


def compute_color_mask(frame: np.ndarray, scale: float = 0.5) -> np.ndarray:
    """Return the cleaned color mask used for detection."""
    if scale != 1.0:
        frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv, BallDetector.HSV_LOWER, BallDetector.HSV_UPPER)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    color_clean = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    return color_clean
        
        