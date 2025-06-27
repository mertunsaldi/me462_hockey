import math
import numpy as np
import time
from typing import List, Optional, Tuple, Union, Dict, Any
from .models import Ball, ArucoMarker, ArucoHitter
from .gadgets import PlotClock


class Scenario:
    """Base class for game scenarios.

    Subclasses should implement :meth:`update` to perform one frame of logic.
    Optional hooks (:meth:`on_start`, :meth:`on_stop`, :meth:`process_message`)
    can be overridden to handle life‑cycle events or ad‑hoc commands.
    """

    finished: bool = False

    def on_start(self) -> None:
        """Called once when the scenario becomes active."""
        pass

    def on_stop(self) -> None:
        """Called once when the scenario is deactivated."""
        pass

    def process_message(self, message: Dict[str, Any]) -> None:
        """Handle a high level control message."""
        pass

    def update(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter]]) -> None:
        raise NotImplementedError

    def get_line_points(self):
        return None

    def get_extra_points(self):
        return None

    def get_extra_labels(self):
        return None


class BallAttacker(Scenario):
    """
    Plan → travel to S → wait → strike.

    * Meeting point  M  lies on the rectangle edge.
    * Start point    S  = M − λ·u(TM)   (λ = BACK_INSIDE_PX).
    * A corridor of width 2·STRIP_W keeps the plan alive.
    """

    # Geometry / behaviour knobs
    BACK_INSIDE_PX = 100.0  # λ
    STRIP_W = 40.0  # corridor half-width (px)
    REACH_PX = 30  # arrival tolerance for S (px)
    speed_tol = 1.0  # ignore slower balls (px/frame)

    # Finite-state phases
    PHASE_TRAVEL = 1
    PHASE_WAIT = 2
    PHASE_STRIKE = 3

    # ------------------------------------------------------------------
    def __init__(self, plotclock, frame_size):
        self.clock = plotclock
        self.w, self.h = frame_size

        # one-time rectangle edges
        self._work_lines = None

        # overlay
        self._ray_line = self._target_line = None
        self._meet_px = self._start_px = None

        # locked plan
        self._locked_M = self._locked_norm = None
        self._start_mm = self._meet_mm = None
        self._dist_SM_mm = None

        # runtime state
        self._phase = None
        self._prev_time = None  # for frame period
        self._cur_speed_ps = None  # puck speed px / s
        self._initialized = False

    def on_start(self) -> None:
        """Reset internal state."""
        self._work_lines = None
        self._locked_M = self._locked_norm = None
        self._phase = None
        self._prev_time = None
        self._cur_speed_ps = None
        self._initialized = False

    # ===============================================================
    def _ray_hits_rectangle(self, bx, by, dx, dy):
        """Return first hit point (mx,my) of ray with rectangle."""
        t_hit, pt_hit = None, None
        for (x1, y1), (x2, y2) in self._work_lines:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x1 - bx) * ey - (y1 - by) * ex) / den
            u = ((x1 - bx) * dy - (y1 - by) * dx) / den
            if t > 1e-6 and 0 <= u <= 1.0:
                if t_hit is None or t < t_hit:
                    t_hit, pt_hit = t, (bx + t * dx, by + t * dy)
        return pt_hit

    # ------------------------------------------------------------------
    def update(self, detections):
        # -- initialization --------------------------------------------
        if not self._initialized:
            if not self.clock.calibration:
                return
            x0, x1 = self.clock.x_range
            y0, y1 = self.clock.y_range
            p00 = self.clock.mm_to_pixel((x0, y0))
            p10 = self.clock.mm_to_pixel((x1, y0))
            p11 = self.clock.mm_to_pixel((x1, y1))
            p01 = self.clock.mm_to_pixel((x0, y1))
            self._work_lines = [(p00, p10), (p10, p11), (p11, p01), (p01, p00)]
            self._initialized = True

        # 2) detections -------------------------------------------------
        ball = next((d for d in detections if isinstance(d, Ball)), None)
        hitter = next((d for d in detections if isinstance(d, ArucoHitter)), None)
        target = next((d for d in detections if isinstance(d, ArucoMarker)), None)
        if not (ball and target):
            return

        # 3) ball kinematics -------------------------------------------
        now = time.time()
        if self._prev_time is None:
            dt = 1 / 30
        else:
            dt = max(1e-4, min(0.1, now - self._prev_time))
        self._prev_time = now

        vx, vy = map(float, ball.velocity)
        speed_pf = (vx * vx + vy * vy) ** 0.5
        if speed_pf < self.speed_tol:
            return
        self._cur_speed_ps = speed_pf / dt  # px / second
        bx, by = map(float, ball.center)
        dx, dy = vx / speed_pf, vy / speed_pf  # unit dir

        # ----------------------------------------------------------------
        # (A) If a plan is locked, run phase machine
        # ----------------------------------------------------------------
        if self._locked_M is not None:
            # corridor test
            d_perp = abs(
                (bx - self._locked_M[0]) * self._locked_norm[0]
                + (by - self._locked_M[1]) * self._locked_norm[1]
            )
            if d_perp > self.STRIP_W:
                self._reset_plan()
            else:
                self._update_phase(ball, hitter)
                return

        # ----------------------------------------------------------------
        # (B) No plan → make a new one
        # ----------------------------------------------------------------
        hit_pt = self._ray_hits_rectangle(bx, by, dx, dy)
        if hit_pt is None:
            return
        mx, my = hit_pt
        self._meet_px = (int(round(mx)), int(round(my)))

        # start point S inside toward target
        tx, ty = map(float, target.center)
        v_TM = np.array([tx - mx, ty - my], dtype=float)
        norm_TM = np.linalg.norm(v_TM)
        if norm_TM < 1e-6:
            return
        u_TM = v_TM / norm_TM
        sx, sy = np.array([mx, my]) - u_TM * self.BACK_INSIDE_PX
        self._start_px = (int(round(sx)), int(round(sy)))

        # convert to mm and range-check
        start_mm = self.clock.find_mm(sx, sy)
        meet_mm = self.clock.find_mm(mx, my)
        if not (start_mm and meet_mm):
            return
        in_x = self.clock.x_range[0] <= start_mm[0] <= self.clock.x_range[1]
        in_y = self.clock.y_range[0] <= start_mm[1] <= self.clock.y_range[1]
        if not (in_x and in_y):
            return

        # send first move toward S
        self.clock.send_command(f"p.setXY({start_mm[0]}, {start_mm[1]})")

        # lock everything
        self._locked_M = np.array([mx, my])
        self._locked_norm = np.array([-dy, dx])  # unit normal
        self._start_mm = start_mm
        self._meet_mm = meet_mm
        self._dist_SM_mm = (
            (start_mm[0] - meet_mm[0]) ** 2 + (start_mm[1] - meet_mm[1]) ** 2
        ) ** 0.5
        self._phase = self.PHASE_TRAVEL

        # overlay
        self._ray_line = ((int(round(bx)), int(round(by))), self._meet_px)
        self._target_line = (self._meet_px, (int(tx), int(ty)))

    # ------------------------------------------------------------------
    # phase machine when a plan is active
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # phase machine when a plan is active
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def _update_phase(self, ball, hitter):
        # still switch TRAVEL → WAIT as before
        if self._phase == self.PHASE_TRAVEL and hitter:
            hx, hy = map(float, hitter.center)
            if (
                (hx - self._start_px[0]) ** 2 + (hy - self._start_px[1]) ** 2
            ) ** 0.5 <= self.REACH_PX:
                self._phase = self.PHASE_WAIT
            return

        # --- WAIT: evaluate score instead of tau test -----------------
        # inside _update_phase, replace the WAIT case
        elif self._phase == self.PHASE_WAIT:
            # --- live kinematics ---
            bx, by = map(float, ball.center)
            dist_px = (
                (bx - self._meet_px[0]) ** 2 + (by - self._meet_px[1]) ** 2
            ) ** 0.5
            speed = self._cur_speed_ps  # px / s

            # --- latency-aware urgency score -------------
            t_lag = 0.05  # s   measured robot delay
            if dist_px <= speed * t_lag:
                self._reset_plan()  # cannot make it, abandon
                return

            v_ref = 30.0  # px / s
            alpha = 1.2  # speed weight
            beta = 1 / 40.0  # distance weight  (≈140 px e-fold)

            score = (1 + alpha * speed / v_ref) * math.exp(
                -beta * (dist_px - speed * t_lag)
            )

            if score >= 1.0:  # θ  — strike threshold
                self.clock.send_command(f"p.setXY({self._meet_mm[0]}, {self._meet_mm[1]})")
                self._phase = self.PHASE_STRIKE

    # ------------------------------------------------------------------
    def _reset_plan(self):
        self._locked_M = self._locked_norm = None
        self._phase = None

    # ------------------------------------------------------------------
    # GUI overlay callbacks
    # ------------------------------------------------------------------
    def get_line_points(self):
        lines = []
        if self._ray_line:
            lines.append(self._ray_line)
        if self._target_line:
            lines.append(self._target_line)
        return lines or None

    def get_extra_points(self):
        pts = []
        if self._meet_px:
            pts.append(self._meet_px)
        if self._start_px:
            pts.append(self._start_px)
        return pts or None

    def get_extra_labels(self):
        labels = []
        if self._meet_px:
            labels.append("Meet P")
        if self._start_px:
            labels.append("Start")
        return labels or None


# ----------------------------------------------------------------------
#  FixedTargetAttacker -- hit incoming ball toward a fixed mm coordinate
# ----------------------------------------------------------------------
class FixedTargetAttacker(BallAttacker):
    """Variant of :class:`BallAttacker` using a fixed (x_mm, y_mm) target."""

    def __init__(
        self, plotclock: PlotClock, frame_size, target_mm: Tuple[float, float]
    ):
        super().__init__(plotclock, frame_size)
        self._target_mm = target_mm

    def on_start(self) -> None:
        super().on_start()

    def update(self, detections):
        # inject a fake ArUco target at the desired coordinates
        if not self.clock.calibration or not self._initialized:
            super().update(detections)
            return

        target_px = self.clock.mm_to_pixel(self._target_mm)
        fake_target = ArucoMarker(-1, [], target_px)
        super().update(detections + [fake_target])


# ----------------------------------------------------------------------
#  BallReflector  v3  — trajectory, meeting point, and Pico “go to” cmd
# ----------------------------------------------------------------------
class BallReflector(Scenario):
    def __init__(
        self,
        plotclock,  # PlotClock instance
        frame_size,  # (width, height)
        speed_tol: float = 1.0,
    ):
        self.clock = plotclock
        self.w, self.h = frame_size
        self.speed_tol = speed_tol

        # state variables
        self._line = None  # ((x1,y1),(x2,y2))
        self._meet_px = None  # (mx, my) in pixels
        self._fired = False  # ensure single strike sequence only once
        self._goal_px = None  # last commanded point in pixels
        self.reach_tol_px = 30  # tolerance (≈ 6–8 mm on a 640×480 frame)
        self.cmd_interval_s = 0.1  # min time between commands
        self._last_cmd_time = 0.0  # unix-time of last setxy
        self._work_lines = None  # list[(p1,p2)…] rectangle edges (px)
        self._initialized = False

    def on_start(self) -> None:
        """Reset internal state."""
        self._line = None
        self._meet_px = None
        self._fired = False
        self._goal_px = None
        self._last_cmd_time = 0.0
        self._work_lines = None
        self._initialized = False

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def update(self, detections):
        # -- initialization --------------------------------------------
        if not self._initialized:
            if not self.clock.calibration:
                return
            x0, x1 = self.clock.x_range
            y0, y1 = self.clock.y_range
            try:
                p00 = self.clock.mm_to_pixel((x0, y0))
                p10 = self.clock.mm_to_pixel((x1, y0))
                p11 = self.clock.mm_to_pixel((x1, y1))
                p01 = self.clock.mm_to_pixel((x0, y1))
                self._work_lines = [(p00, p10), (p10, p11), (p11, p01), (p01, p00)]
            except RuntimeError:
                self._work_lines = []
            self._initialized = True

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
            if 0 <= by + t * dy <= self.h - 1:
                t_vals.append(t)
        elif dx < 0:
            t = -bx / dx
            if 0 <= by + t * dy <= self.h - 1:
                t_vals.append(t)
        if dy > 0:
            t = (self.h - 1 - by) / dy
            if 0 <= bx + t * dx <= self.w - 1:
                t_vals.append(t)
        elif dy < 0:
            t = -by / dy
            if 0 <= bx + t * dx <= self.w - 1:
                t_vals.append(t)

        pos_ts = [t for t in t_vals if t > 1e-6]
        if not pos_ts:
            return
        t_edge = min(pos_ts)

        ex, ey = bx + dx * t_edge, by + dy * t_edge
        self._line = (
            (int(round(bx)), int(round(by))),
            (int(round(ex)), int(round(ey))),
        )

        # ── 4. Meeting point ─────────────────────────────────────────
        hx, hy = map(float, hitter.center)
        t_hit = max(0, min((hx - bx) * dx + (hy - by) * dy, t_edge))
        mx, my = bx + dx * t_hit, by + dy * t_hit
        self._meet_px = (int(round(mx)), int(round(my)))

        # ── 5. Convert to mm & range-check ───────────────────────────
        meet_mm = self.clock.find_mm(mx, my)
        if not meet_mm:
            return
        if not (
            self.clock.x_range[0] <= meet_mm[0] <= self.clock.x_range[1]
            and self.clock.y_range[0] <= meet_mm[1] <= self.clock.y_range[1]
        ):
            return  # outside workspace

        # ── 6. Decide if we should issue a new setxy ─────────────────
        now = time.time()
        time_ok = (now - self._last_cmd_time) >= self.cmd_interval_s

        goal_reached = False
        if self._goal_px is not None:
            gx, gy = self._goal_px
            goal_reached = (hx - gx) ** 2 + (hy - gy) ** 2 <= self.reach_tol_px**2

        if self._goal_px is None or goal_reached and time_ok:
            self.clock.send_command(f"p.setXY({meet_mm[0]}, {meet_mm[1]})")
            self._goal_px = self._meet_px
            self._last_cmd_time = now

    # ------------------------------------------------------------------
    def get_line_points(self):
        return [self._line] if self._line else None

    def get_extra_points(self):
        return [self._meet_px] if self._meet_px else None

    def get_extra_labels(self):
        return ["Meet P"] if self._meet_px else None


class StandingBallHitter(Scenario):
    """Calibrates PlotClock, then strikes the ball toward the target."""

    def __init__(self, plotclock: PlotClock):
        self.plotclock = plotclock
        self.hitter: Optional[ArucoHitter] = None
        self.target: Optional[ArucoMarker] = None
        self.ball: Optional[Ball] = None
        self._line: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None
        self._start_px: Optional[Tuple[int, int]] = None
        self._start_mm: Optional[Tuple[float, float]] = None
        self._fired = False  # ensure we send move-strike sequence only once

    def on_start(self) -> None:
        """Reset internal state when the scenario begins."""
        self._fired = False

    # ------------------------------------------------------------------
    def update(self, detections):
        # 1) require calibration first
        if not self.plotclock.calibration:
            return  # nothing else until calibrated

        # 2) reset per‑frame state
        self.hitter = self.target = self.ball = None
        self._line = self._start_px = self._start_mm = None

        for d in detections:
            if isinstance(d, ArucoHitter):
                self.hitter = d
            elif isinstance(d, ArucoMarker):
                self.target = d
            elif isinstance(d, Ball):
                self.ball = d

        if self.ball and self.target:
            bx, by = self.ball.center
            tx, ty = self.target.center
            self._line = ((bx, by), (tx, ty))

            # start point 60 px behind ball along target line
            dx, dy = tx - bx, ty - by
            dist = np.hypot(dx, dy)
            if dist > 0:
                sx = int(bx - dx / dist * 70)
                sy = int(by - dy / dist * 70)
                self._start_px = (sx, sy)
                self._start_mm = self.plotclock.find_mm(sx, sy)

                # once per run: move hitter then strike
                if not self._fired and self._start_mm:
                    ball_mm = self.clock.find_mm(bx, by)
                    self.clock.send_command(
                        f"p.setXY({self._start_mm[0]}, {self._start_mm[1]})"
                    )
                    time.sleep(0.7)
                    self.clock.send_command(
                        f"p.setXY({ball_mm[0]}, {ball_mm[1]})"
                    )
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