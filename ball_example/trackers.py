import cv2
import math
import threading
import numpy as np
from typing import Dict, List, Tuple, Optional
from .models import Ball
from .detectors import BallDetector

# ---------- Tunable thresholds ----------
SPATIAL_THRESHOLD   = 60.0     # px centroid distance to match / quick‑add check
COLOR_THRESHOLD     = 90.0     # BGR distance for colour verification
VELOCITY_THRESHOLD  = 1.0      # px/frame – below → 0
MISSING_THRESHOLD   = 6        # consecutive misses to drop a ball
REINIT_INTERVAL     = 70       # frames between *forced* re‑detects
DT                  = 1.0      # Kalman Δt (frames)
INTERSECT_TOLERANCE = 10.0      # allowed overlap between balls
PATCH_HALF          = 2        # 5×5 colour patch (radius)
# Run detection on a scaled-down image to reduce CPU load
DETECTION_SCALE    = 0.5
# Run detection on a scaled-down image to reduce CPU load
DETECTION_SCALE    = 0.5
# ---------------------------------------

# ---------- Kalman helper --------------
class KalmanTracker:
    """Simple [x, y, vx, vy] ↔ [x, y] Kalman filter."""
    def __init__(self, init_pos: Tuple[int, int], dt: float = DT):
        k = cv2.KalmanFilter(4, 2)
        k.transitionMatrix   = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], np.float32)
        k.measurementMatrix  = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        k.processNoiseCov    = np.eye(4,  dtype=np.float32) * 1e-2
        k.measurementNoiseCov= np.eye(2,  dtype=np.float32) * 1e-1
        x, y = init_pos
        k.statePre  = np.array([[x],[y],[0],[0]], np.float32)
        k.statePost = k.statePre.copy()
        self.kf = k
    def predict(self):                return self.kf.predict()
    def correct(self, p: Tuple[int,int]): return self.kf.correct(np.array([[np.float32(p[0])],[np.float32(p[1])]]))
# ---------------------------------------

class BallTracker:
    """CSRT + Kalman multi‑ball tracker with colour verification & quick‑add."""
        # keep as attribute for backward compatibility
    DETECTION_SCALE = DETECTION_SCALE
    def __init__(self):
        self.trackers: Dict[str, cv2.TrackerCSRT] = {}
        self.kalman  : Dict[str, KalmanTracker]   = {}
        self.balls   : Dict[str, Ball]            = {}
        self.missing : Dict[str, int]             = {}
        self.next_id = 0
        self.frame_count = 0
        self.lock = threading.Lock()

    # ----------- utility helpers ----------
    @staticmethod
    def _mean_patch(frame: np.ndarray, pos: Tuple[float,float]) -> np.ndarray:
        h,w = frame.shape[:2]
        x = int(np.clip(pos[0], PATCH_HALF, w-PATCH_HALF-1))
        y = int(np.clip(pos[1], PATCH_HALF, h-PATCH_HALF-1))
        patch = frame[y-PATCH_HALF:y+PATCH_HALF+1, x-PATCH_HALF:x+PATCH_HALF+1]
        return patch.mean(axis=(0,1))

    def _reset_trackers(self, frame: np.ndarray):
        """(Re)initialise OpenCV trackers for the currently known balls.

        Caller must hold ``self.lock``.
        """
        self.trackers.clear()
        self.kalman.clear()
        for bid, b in self.balls.items():
            x, y = b.center
            r = b.radius
            t = cv2.TrackerCSRT_create()
            t.init(frame, (x - r, y - r, 2 * r, 2 * r))
            self.trackers[bid] = t
            self.kalman[bid] = KalmanTracker(b.center)
            self.missing[bid] = 0


    def force_redetect(self, frame: np.ndarray, mask: Optional[np.ndarray] = None):
        """Clear all state and redetect balls using current parameters."""
        with self.lock:
            self.trackers.clear()
            self.kalman.clear()
            self.missing.clear()
            self.balls.clear()
            self.next_id = 0
            for d in BallDetector.detect(frame, mask=mask, scale=DETECTION_SCALE):
                bid = f"ball_{self.next_id}"
                self.next_id += 1
                d.id = bid
                self.balls[bid] = d
            self._reset_trackers(frame)
            self.frame_count = 0
        

    # ---------------- main update ----------------
    def update(self, frame: np.ndarray, mask: Optional[np.ndarray] = None) -> List[Ball]:
        with self.lock:
            self.frame_count += 1

            # ------- if nothing tracked, detect now -------
            if not self.balls:
                for d in BallDetector.detect(frame, mask=mask, scale=DETECTION_SCALE):
                    bid = f"ball_{self.next_id}"
                    self.next_id += 1
                    d.id = bid
                    self.balls[bid] = d
                self._reset_trackers(frame)
    def update(self, frame: np.ndarray) -> List[Ball]:
        self.frame_count += 1
        # ------- if nothing tracked, detect now -------
        if not self.balls:
            for d in BallDetector.detect(frame, scale=DETECTION_SCALE):
                bid=f"ball_{self.next_id}"; self.next_id+=1; d.id=bid; self.balls[bid]=d
            self._reset_trackers(frame)

            updates: Dict[str, Ball] = {}

            # ------- update existing trackers ------------
            for bid, tr in list(self.trackers.items()):
                old_ball = self.balls.get(bid)
                if old_ball is None:
                    continue  # safety
                ok, bbox = tr.update(frame)
                if not ok:
                    self.missing[bid] = self.missing.get(bid, 0) + 1
                    if self.missing[bid] >= MISSING_THRESHOLD:
                        # drop completely
                        self.trackers.pop(bid, None)
                        self.kalman.pop(bid, None)
                        self.balls.pop(bid, None)
                        self.missing.pop(bid, None)
                    continue
                cx = int(bbox[0] + bbox[2] / 2)
                cy = int(bbox[1] + bbox[3] / 2)
                self.kalman[bid].predict()
                xk, yk, vx, vy = self.kalman[bid].correct((cx, cy)).flatten()

                # colour verification using stored colour BEFORE any popping
                stored_color = np.array(old_ball.color, np.float32)
                if np.linalg.norm(self._mean_patch(frame, (xk, yk)) - stored_color) > COLOR_THRESHOLD:
                    # colour mismatch → treat as miss this frame
                    self.missing[bid] = self.missing.get(bid, 0) + 1
                    if self.missing[bid] >= MISSING_THRESHOLD:
                        self.trackers.pop(bid, None)
                        self.kalman.pop(bid, None)
                        self.balls.pop(bid, None)
                        self.missing.pop(bid, None)
                    continue

                if math.hypot(vx, vy) < VELOCITY_THRESHOLD:
                    vx, vy = 0.0, 0.0

                b = Ball(center=(int(xk), int(yk)), radius=old_ball.radius, color=old_ball.color, ball_id=bid)
                b.velocity = (vx, vy)
                updates[bid] = b
                self.missing[bid] = 0

            self.balls = updates

            # ------- quick‑add new balls every frame ------
            for d in BallDetector.detect(frame, mask=mask, scale=DETECTION_SCALE):
                # skip if near existing ball
                if any(math.hypot(d.center[0] - b.center[0], d.center[1] - b.center[1]) < SPATIAL_THRESHOLD for b in self.balls.values()):
                    continue
                # skip if overlaps existing ball
                if any(math.hypot(d.center[0] - b.center[0], d.center[1] - b.center[1]) < (d.radius + b.radius - INTERSECT_TOLERANCE) for b in self.balls.values()):
                    continue
                bid = f"ball_{self.next_id}"
                self.next_id += 1
                d.id = bid
                # start tracker & kalman
                x, y = d.center
                r = d.radius
                t = cv2.TrackerCSRT_create()
                t.init(frame, (x - r, y - r, 2 * r, 2 * r))
                self.trackers[bid] = t
                self.kalman[bid] = KalmanTracker(d.center)
                self.missing[bid] = 0
                self.balls[bid] = d
                updates[bid] = d
        # ------- quick‑add new balls every frame ------
        for d in BallDetector.detect(frame, scale=DETECTION_SCALE):
            # skip if near existing ball
            if any(math.hypot(d.center[0]-b.center[0], d.center[1]-b.center[1]) < SPATIAL_THRESHOLD for b in self.balls.values()):
                continue
            # skip if overlaps existing ball
            if any(math.hypot(d.center[0]-b.center[0], d.center[1]-b.center[1]) < (d.radius+b.radius-INTERSECT_TOLERANCE) for b in self.balls.values()):
                continue
            bid=f"ball_{self.next_id}"; self.next_id+=1; d.id=bid
            # start tracker & kalman
            x,y=d.center; r=d.radius
            t=cv2.TrackerCSRT_create(); t.init(frame,(x-r,y-r,2*r,2*r))
            self.trackers[bid]=t; self.kalman[bid]=KalmanTracker(d.center); self.missing[bid]=0
            self.balls[bid]=d; updates[bid]=d

            # -------- forced re‑detect occasionally -------
            if self.frame_count >= REINIT_INTERVAL:
                self.frame_count = 0
                # run detection and merge with existing state (simple add if distinct)
                for d in BallDetector.detect(frame, mask=mask, scale=DETECTION_SCALE):
                    if any(math.hypot(d.center[0] - b.center[0], d.center[1] - b.center[1]) < SPATIAL_THRESHOLD for b in self.balls.values()):
                        continue
                    bid = f"ball_{self.next_id}"
                    self.next_id += 1
                    d.id = bid
                    self.balls[bid] = d
                self._reset_trackers(frame)
        # -------- forced re‑detect occasionally -------
        if self.frame_count >= REINIT_INTERVAL:
            self.frame_count = 0
            # run detection and merge with existing state (simple add if distinct)
            for d in BallDetector.detect(frame, scale=DETECTION_SCALE):
                if any(math.hypot(d.center[0]-b.center[0], d.center[1]-b.center[1]) < SPATIAL_THRESHOLD for b in self.balls.values()):
                    continue
                bid=f"ball_{self.next_id}"; self.next_id+=1; d.id=bid
                self.balls[bid]=d
            self._reset_trackers(frame)

            return list(self.balls.values())
