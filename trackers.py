import cv2
import numpy as np
from typing import Dict, List
from models import Ball
from detectors import detect_balls


class TrackerManager:
    """
    Manages persistent IDs and OpenCV trackers for multiple balls.
    - Uses CSRT trackers for frame-to-frame tracking.
    - Re-detects periodically, matching detections to existing IDs.
    - Immediately removes balls when trackers fail (i.e., ball leaves arena).
    """
    def __init__(
        self,
        reinit_interval: int = 30,
        max_match_distance: float = 50.0,
        immediate_remove: bool = True
    ):
        # Active trackers per ball ID
        self.trackers: Dict[str, cv2.TrackerCSRT] = {}
        # Ball states per ID
        self.balls: Dict[str, Ball] = {}
        self.reinit_interval = reinit_interval
        self.frame_count = 0
        self.max_match_distance = max_match_distance
        self.immediate_remove = immediate_remove

    def _match_detections(self, detections: List[Ball]) -> None:
        """
        Match new detections to existing balls by proximity to preserve IDs.
        """
        new_balls: Dict[str, Ball] = {}
        ids = list(self.balls.keys())
        centers = np.array([self.balls[i].center for i in ids]) if ids else np.empty((0,2))

        for det in detections:
            if centers.size > 0:
                dists = np.linalg.norm(centers - np.array(det.center), axis=1)
                idx = int(np.argmin(dists))
                if dists[idx] <= self.max_match_distance:
                    det.id = ids[idx]
                    new_balls[det.id] = det
                    continue
            # new ball
            new_balls[det.id] = det

        # update mapping
        self.balls = new_balls

    def _reset_trackers(self, frame) -> None:
        """
        Initialize CSRT trackers for all current balls.
        """
        self.trackers.clear()
        for ball_id, ball in self.balls.items():
            x, y = ball.center
            r = ball.radius
            bbox = (x - r, y - r, 2 * r, 2 * r)
            tracker = cv2.TrackerCSRT_create()
            tracker.init(frame, bbox)
            self.trackers[ball_id] = tracker

    def update(self, frame) -> List[Ball]:
        """
        Update trackers, immediately removing balls when tracking fails,
        and periodically re-detect and rematch IDs.
        Returns a list of current Ball objects with consistent IDs.
        """
        self.frame_count += 1

        # 1. Update existing trackers
        for ball_id, tracker in list(self.trackers.items()):
            ok, bbox = tracker.update(frame)
            if not ok:
                # Remove ball immediately when lost
                if self.immediate_remove:
                    self.trackers.pop(ball_id)
                    self.balls.pop(ball_id, None)
                continue
            # Update ball center
            x, y, w, h = bbox
            cx, cy = int(x + w/2), int(y + h/2)
            r = int(max(w, h)/2)
            color = self.balls[ball_id].color
            self.balls[ball_id] = Ball(center=(cx, cy), radius=r, color=color, ball_id=ball_id)

        # 2. Periodic re-detection
        if self.frame_count >= self.reinit_interval:
            detections = detect_balls(frame)
            self._match_detections(detections)
            self._reset_trackers(frame)
            self.frame_count = 0

        # Return all current balls
        return list(self.balls.values())
