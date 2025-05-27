import numpy as np
import time
from typing import List, Optional, Tuple, Union, Dict, Any
from models import Ball, ArucoMarker, ArucoHitter
from gadgets import PlotClock


class Scenario:
    """Base class for game scenarios."""

    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        raise NotImplementedError

    def get_line_points(self):
        return None

    def get_extra_points(self):
        return None

    def get_extra_labels(self):
        return None


class BallAttacker(Scenario):

    # to-do:
    # topun velocity vectörünün açısına göre bir treshold koy
    # threshold içindeysek ve hitter start pointe hareket etmişse, start point ve meeting pointi lockla, tekrardan hesaplanmasın
    # eğer topun geliş açısı drastik değişirse (threshold dışındaysa), unlockla ve yeni start ve meet pointler hesapla, hitter starta gitsin


    def __init__(self, plotclock, frame_size, speed_tol: float = 1.0):
        self.clock = plotclock
        self.w, self.h = frame_size
        self.speed_tol = speed_tol
        # overlay state
        self._work_lines = None
        self._line = None            # trajectory
        self._target_line = None     # target↔meet
        self._meet_px = None
        self._init_px = None
        # command / timing
        self._goal_px = None
        self.reach_tol_px = 10
        self.cmd_interval_s = 0.1
        self._last_cmd_time = 0.0

    # ------------------------------------------------------------------
    def update(self, detections):
        # 0) calibration
        if not self.clock.calibration:
            self.clock.calibrate(detections)
            return

        # 1) working rectangle (once)
        if self._work_lines is None:
            x0, x1 = self.clock.x_range
            y0, y1 = self.clock.y_range
            p00 = self.clock.mm_to_pixel((x0, y0))
            p10 = self.clock.mm_to_pixel((x1, y0))
            p11 = self.clock.mm_to_pixel((x1, y1))
            p01 = self.clock.mm_to_pixel((x0, y1))
            self._work_lines = [(p00, p10), (p10, p11),
                                (p11, p01), (p01, p00)]

        # 2) reset per-frame overlays
        self._line = self._target_line = self._meet_px = self._init_px = None

        ball    = next((d for d in detections if isinstance(d, Ball)), None)
        hitter  = next((d for d in detections if isinstance(d, ArucoHitter)), None)
        target  = next((d for d in detections if isinstance(d, ArucoMarker)), None)
        if not (ball and hitter and target):
            return

        # 3) use ball.velocity directly
        vx, vy = map(float, ball.velocity)
        speed  = (vx * vx + vy * vy) ** 0.5
        if speed < self.speed_tol:
            return
        dx, dy = vx / speed, vy / speed
        bx, by = map(float, ball.center)

        # 3a trajectory line to frame edge
        t_vals = []
        if dx > 0:
            t_vals.append((self.w - 1 - bx) / dx)
        elif dx < 0:
            t_vals.append(-bx / dx)
        if dy > 0:
            t_vals.append((self.h - 1 - by) / dy)
        elif dy < 0:
            t_vals.append(-by / dy)
        pos_ts = [t for t in t_vals if t > 1e-6]
        if not pos_ts:
            return
        t_edge = min(pos_ts)
        ex, ey = bx + dx * t_edge, by + dy * t_edge
        self._line = ((int(round(bx)), int(round(by))),
                      (int(round(ex)), int(round(ey))))

        # 4) meeting point: projection of hitter on trajectory
        hx, hy = map(float, hitter.center)
        t_hit = max(0, min((hx - bx) * dx + (hy - by) * dy, t_edge))
        mx, my = bx + dx * t_hit, by + dy * t_hit
        self._meet_px = (int(round(mx)), int(round(my)))

        # 5) Target ↔ Meet line
        tx, ty = map(int, target.center)
        self._target_line = ((tx, ty), self._meet_px)

        # 6) initial point 70 px beyond MEET along the Target→Meet axis
        tx, ty = map(float, target.center)
        mx, my = np.array([mx, my]) - np.array([dx, dy], dtype=float) * 50.0
        self._meet_px = (int(round(mx)), int(round(my)))
        v_t = np.array([mx - tx, my - ty], dtype=float)  # target → meet
        norm_t = np.linalg.norm(v_t)
        if norm_t < 1e-6:
            return  # degenerate, skip frame
        u_t = v_t / norm_t  # unit direction
        ix, iy = np.array([mx, my]) + u_t * 50.0  # 70 px past meet
        self._init_px = (int(round(ix)), int(round(iy)))


        # 7) command logic (init point)
        init_mm = self.clock.find_mm(ix, iy)
        if not init_mm:
            return
        if not (self.clock.x_range[0] <= init_mm[0] <= self.clock.x_range[1] and
                self.clock.y_range[0] <= init_mm[1] <= self.clock.y_range[1]):
            return

        now = time.time()
        time_ok = (now - self._last_cmd_time) >= self.cmd_interval_s

        goal_reached = False
        if self._goal_px is not None:
            gx, gy = self._goal_px
            goal_reached = ((hx - gx) ** 2 + (hy - gy) ** 2
                            <= self.reach_tol_px ** 2)

        if self._goal_px is None or (goal_reached and time_ok):
            self.clock.send_command('setxy', *init_mm)
            self._goal_px = self._init_px
            self._last_cmd_time = now

    # ------------------------------------------------------------------
    def get_line_points(self):
        lines = []
        if self._work_lines:
            lines.extend(self._work_lines)
        if self._line:
            lines.append(self._line)
        if self._target_line:
            lines.append(self._target_line)
        return lines or None

    def get_extra_points(self):
        pts = []
        if self._meet_px:  pts.append(self._meet_px)
        if self._init_px:  pts.append(self._init_px)
        return pts or None

    def get_extra_labels(self):
        labels = []
        if self._meet_px:  labels.append("Meet P")
        if self._init_px:  labels.append("Start")
        return labels or None


# ----------------------------------------------------------------------
#  BallReflector  v3  — trajectory, meeting point, and Pico “go to” cmd
# ----------------------------------------------------------------------
class BallReflector(Scenario):
    def __init__(self,
                 plotclock,                     # PlotClock instance
                 frame_size,                    # (width, height)
                 speed_tol: float = 1.0):
        self.clock = plotclock
        self.w, self.h = frame_size
        self.speed_tol = speed_tol

        # state variables
        self._line       = None                 # ((x1,y1),(x2,y2))
        self._meet_px    = None                 # (mx, my) in pixels
        self._fired      = False                # ensure single drawto
        self._goal_px = None  # last commanded point in pixels
        self.reach_tol_px = 30  # tolerance (≈ 6–8 mm on a 640×480 frame)
        self.cmd_interval_s = 0.1  # min time between commands
        self._last_cmd_time = 0.0  # unix-time of last setxy
        self._work_lines = None  # list[(p1,p2)…] rectangle edges (px)

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def update(self, detections):
        # ── 0. Calibration ───────────────────────────────────────────
        if not self.clock.calibration:
            self.clock.calibrate(detections)
            return

        # ── 1. Build work-area rectangle once ───────────────────────
        if self._work_lines is None:
            x0, x1 = self.clock.x_range
            y0, y1 = self.clock.y_range
            try:
                p00 = self.clock.mm_to_pixel((x0, y0))
                p10 = self.clock.mm_to_pixel((x1, y0))
                p11 = self.clock.mm_to_pixel((x1, y1))
                p01 = self.clock.mm_to_pixel((x0, y1))
                self._work_lines = [(p00, p10), (p10, p11),
                                    (p11, p01), (p01, p00)]
            except RuntimeError:
                self._work_lines = []

        # ── 2. Reset overlay state every frame ──────────────────────
        self._line = self._meet_px = None

        ball = next((d for d in detections if isinstance(d, Ball)), None)
        hitter = next((d for d in detections if isinstance(d, ArucoHitter)), None)
        if not (ball and hitter):
            return

        # ── 3. Build trajectory to frame edge ───────────────────────
        vx, vy = map(float, ball.velocity)
        speed = (vx * vx + vy * vy) ** 0.5
        if speed < self.speed_tol:
            return

        bx, by = map(float, ball.center)
        dx, dy = vx / speed, vy / speed

        t_vals = []
        if dx > 0:
            t = (self.w - 1 - bx) / dx
            if 0 <= by + t * dy <= self.h - 1: t_vals.append(t)
        elif dx < 0:
            t = -bx / dx
            if 0 <= by + t * dy <= self.h - 1: t_vals.append(t)
        if dy > 0:
            t = (self.h - 1 - by) / dy
            if 0 <= bx + t * dx <= self.w - 1: t_vals.append(t)
        elif dy < 0:
            t = -by / dy
            if 0 <= bx + t * dx <= self.w - 1: t_vals.append(t)

        pos_ts = [t for t in t_vals if t > 1e-6]
        if not pos_ts:
            return
        t_edge = min(pos_ts)

        ex, ey = bx + dx * t_edge, by + dy * t_edge
        self._line = ((int(round(bx)), int(round(by))),
                      (int(round(ex)), int(round(ey))))

        # ── 4. Meeting point ─────────────────────────────────────────
        hx, hy = map(float, hitter.center)
        t_hit = max(0, min((hx - bx) * dx + (hy - by) * dy, t_edge))
        mx, my = bx + dx * t_hit, by + dy * t_hit
        self._meet_px = (int(round(mx)), int(round(my)))

        # ── 5. Convert to mm & range-check ───────────────────────────
        meet_mm = self.clock.find_mm(mx, my)
        if not meet_mm:
            return
        if not (self.clock.x_range[0] <= meet_mm[0] <= self.clock.x_range[1] and
                self.clock.y_range[0] <= meet_mm[1] <= self.clock.y_range[1]):
            return  # outside workspace

        # ── 6. Decide if we should issue a new setxy ─────────────────
        now = time.time()
        time_ok = (now - self._last_cmd_time) >= self.cmd_interval_s

        goal_reached = False
        if self._goal_px is not None:
            gx, gy = self._goal_px
            goal_reached = ((hx - gx) ** 2 + (hy - gy) ** 2
                            <= self.reach_tol_px ** 2)

        if self._goal_px is None or goal_reached and time_ok:
            self.clock.send_command('setxy', *meet_mm)
            self._goal_px = self._meet_px
            self._last_cmd_time = now

    # ------------------------------------------------------------------
    def get_line_points(self):
        # rectangle edges + current trajectory line
        if self._work_lines:
            return self._work_lines + ([self._line] if self._line else [])
        return self._line

    def get_extra_points(self):
        return [self._meet_px] if self._meet_px else None

    def get_extra_labels(self):
        return ["Meet P"] if self._meet_px else None


class StandingBallHitter(Scenario):
    """Calibrates PlotClock, then strikes the ball toward the target."""

    def __init__(self, plotclock: PlotClock):
        self.clock = plotclock
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball:   Optional[Ball] = None
        self._line:       Optional[Tuple[Tuple[int,int],Tuple[int,int]]] = None
        self._start_px:   Optional[Tuple[int,int]] = None
        self._start_mm:   Optional[Tuple[float,float]] = None
        self._fired = False          # ensure we send drawto sequence only once

    # ------------------------------------------------------------------
    def update(self, detections):
        # 1) drive calibration first
        if not self.clock.calibration:
            self.clock.calibrate(detections)
            return   # nothing else until calibrated

        # 2) reset per‑frame state
        self.hitter = self.target = self.ball = None
        self._line = self._start_px = self._start_mm = None

        for d in detections:
            if isinstance(d, ArucoHitter):   self.hitter = d
            elif isinstance(d, ArucoMarker): self.target  = d
            elif isinstance(d, Ball):        self.ball    = d

        if self.ball and self.target:
            bx,by = self.ball.center
            tx,ty = self.target.center
            self._line = ((bx,by),(tx,ty))

            # start point 60 px behind ball along target line
            dx,dy = tx-bx, ty-by
            dist = np.hypot(dx,dy)
            if dist>0:
                sx = int(bx - dx/dist*70)
                sy = int(by - dy/dist*70)
                self._start_px = (sx,sy)
                self._start_mm = self.clock.find_mm(sx,sy)

                # once per run: move hitter then strike
                if not self._fired and self._start_mm:
                    ball_mm = self.clock.find_mm(bx,by)
                    self.clock.send_command('drawto', *self._start_mm)
                    time.sleep(0.7)
                    self.clock.send_command('setxy', *ball_mm)
                    self._fired = True

    # ------------------------------------------------------------------
    def get_line_points(self):
        return self._line

    def get_extra_points(self):
        return [self._start_px] if self._start_px else None

    def get_extra_labels(self):
        if not self._start_px:
            return None
        if self._start_mm:
            return [f"Start ({self._start_mm[0]:.1f},{self._start_mm[1]:.1f} mm)"]
        return ["Start"]