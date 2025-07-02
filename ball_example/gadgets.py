import threading
import sys
from typing import Optional, Any, Dict, List, Tuple, Union
import serial
import serial.tools.list_ports
import time
import math
import numpy as np
from .models import Ball, Obstacle, ArucoMarker, ArucoHitter, ArucoManager, Arena

# Calibration move commands currently drive the clock very close to the
# workspace limits which may lead to missed detections if the arm hits the
# boundary.  Use a small margin so calibration points stay inside the
# reachable area a bit more.
#
# ``CAL_MARGIN_SCALE`` expresses this margin relative to the mechanism's
# first link length ``L1``.  The default value preserves the previous
# behaviour of a 10 mm margin when ``L1`` is 35 mm.
CAL_MARGIN_SCALE = 7.0 / 35.0


class Gadgets:
    """
    Base class for gadget communications.

    Attributes:
        port: RS232/serial port identifier (e.g. '/dev/ttyUSB0') or None
        baudrate: communication baud rate
        timeout: read/write timeout in seconds
        commands: dictionary of supported command strings
    """
    def __init__(
        self,
        port: Optional[str] = None,
        *,
        baudrate: int = 115200,
        timeout: float = 20
    ):
        self.port: Optional[str] = port
        self.baudrate: int = baudrate
        self.timeout: float = timeout
        self.commands: Dict[str, str] = {}
        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._interactive: bool = False

    def _find_port(self) -> Optional[str]:
        """
        Auto-detect a Pico-like serial port.
        """
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) in {(0x2E8A, 0x0005), (0x2E8A, 0x000A)} or "Pico" in p.description:
                return p.device
        return None

    def start_comms(self) -> None:
        """
        Initialize communications: open serial port and start reader thread.
        """
        if self._ser and self._ser.is_open:
            return
        port = self.port or self._find_port()
        if not port:
            raise RuntimeError("Could not auto-detect serial port; specify port explicitly.")
        try:
            self._ser = serial.Serial(port, baudrate=self.baudrate, timeout=self.timeout)
            # allow device reset
            time.sleep(2)
            self._start_reader()
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}") from e

    def _reader(self) -> None:
        """
        Background thread reading lines from device and printing them.
        """
        while self._ser and self._ser.is_open:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                text = line.decode('utf-8', 'replace').rstrip()
                sys.stdout.write(f"\r\n{text}\n")
                if self._interactive:
                    sys.stdout.write("> ")
                sys.stdout.flush()
            except (serial.SerialException, OSError):
                break
            except UnicodeDecodeError:
                continue

    def _start_reader(self) -> None:
        if not self._reader_thread or not self._reader_thread.is_alive():
            self._reader_thread = threading.Thread(target=self._reader, daemon=True)
            self._reader_thread.start()

    def send_command(self, cmd_name: str, *params: Any) -> None:
        """
        Send a formatted command or raw string to the device. Use commands dict or raw.
        """
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port not open; call start_comms() first.")
        if cmd_name in self.commands:
            fmt = self.commands[cmd_name]
            cmd_str = fmt.format(*params)
        else:
            # treat cmd_name as raw full command
            cmd_str = cmd_name
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
        self._ser.write(cmd_str.encode('utf-8'))
        self._ser.flush()

    def close(self) -> None:
        """
        Close the serial connection.
        """
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None


from .master_pico import MasterPico


class PlotClock(Gadgets):
    """
    Gadget for driving a PlotClock ("vurucu5000") firmware over serial.

    It now owns an **internal calibration FSM** – no external PlotClockCalibration
    scenario needed.  Call `calibrate(detections)` every frame until it returns a
    non‑None dict, which is also stored in `self.calibration`.
    """

    # ──────────────────────────────────────────────────────────
    # Construction / serial helpers
    # ──────────────────────────────────────────────────────────
    calibration_marker_cls = ArucoHitter
    # Default idle location within the workspace
    WAIT_POS_MM: Tuple[float, float] | None = (0.0, 60.0)

    def __init__(
        self,
        *,
        device_id: Optional[int] = None,
        master: Optional[MasterPico] = None,
        cal_margin_scale: float = CAL_MARGIN_SCALE,
    ):
        super().__init__(None, baudrate=115200, timeout=20)
        self.device_id = device_id
        self.master = master
        self.cal_margin_scale = cal_margin_scale
        self.commands = {
        #istersek buraya commandlar ekleyebiliriz
        "mode": "mode {0}",    
        }
        # Obtain working space geometry directly from the attached PlotClock if
        # possible.  ``getMaxX`` and ``getMinY`` give the horizontal limits and
        # lower bound.  The mechanical link lengths are fetched via ``getL1``,
        # ``getL2`` and ``getL3()``.  The union of two disk segments defined by
        # these lengths describes the reachable area.  If any query fails,
        # sensible defaults are used.

        max_x = min_y = None
        length = dist = None
        l1 = None
        if self.master is not None and self.device_id is not None:
            try:
                max_x = float(self._query_value("p.getMaxX()"))
                min_y = float(self._query_value("p.getMinY()"))
                l1 = float(self._query_value("p.getL1()"))
                l2 = float(self._query_value("p.getL2()"))
                # distance between the two servo origins (L3 in the
                # firmware) is divided by two because the origin of the
                # coordinate system is at the midpoint.
                l3 = float(self._query_value("p.getL3()"))
                length = l1 + l2
                dist = l3 / 2.0
            except Exception:
                max_x = min_y = length = dist = None

        if max_x is not None and min_y is not None and length is not None and dist is not None:
            self.max_x = max_x
            self.min_y = min_y
            self.workspace_radius = length
            self.workspace_dist = dist
            self.l1 = l1
            min_x = -max_x
            max_y = math.sqrt(max(0.0, length * length - dist * dist))
            self.x_range = (min_x, max_x)
            self.y_range = (min_y, max_y)
        else:
            if self.device_id == 1:
                self.max_x = 110.5313
                self.min_y = 40.0
                self.workspace_radius = 65+95
                self.workspace_dist = 25/2
                self.l1 = 65.0
            else:
                self.max_x = 365.0827
                self.min_y = 108.0
                self.workspace_radius = 270+328
                self.workspace_dist = 75.0
                self.l1 = 270
            self.x_range = (-self.max_x, self.max_x)
            self.y_range = (self.min_y, math.sqrt(max(0.0, self.workspace_radius ** 2 - self.workspace_dist ** 2)))

        # Calibration state --------------------------------------------------
        self._cal_state: int = 0          # 0=idle,1..n=fsm
        span_x = 2 * self.max_x
        span_y = self.y_range[1] - self.min_y
        self._axis_len = 0.5 * min(span_x, span_y)
        base_x = 0
        base_y = self.min_y

        self.cal_margin_mm = self.cal_margin_scale * self.l1

        # Keep calibration points away from the edges -----------------
        m = self.cal_margin_mm



        # self._mm_pts = [
        #     (base_x + m, base_y + self._axis_len - m),
        #     (base_x + m, base_y + m),
        #     (-base_x, base_y + m),]
        
        self._mm_pts = [
            (base_x + m, base_y + m),
            (base_x + m, base_y + self._axis_len - m),
            (-base_x - m, base_y + self._axis_len - m)]
        


        # self._mm_pts = [
        #     (base_x + m, base_y + self._axis_len - 1.8*m),
        #     (base_x + m, base_y + self._axis_len + 1.8*m),
        #     (-base_x - m, base_y + self._axis_len + 1.8*m),]

        self._px_hits: List[Tuple[int,int]] = []
        self._last_cmd_t: float = 0.0
        self._delay: float = 2.0          # seconds between moves
        # Results
        self.calibration: Optional[Dict[str,Any]] = None  # u_x,u_y,origin
        self._u_x: Optional[np.ndarray] = None
        self._u_y: Optional[np.ndarray] = None
        self._origin_px: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    def send_command(self, cmd_name: str, *params: Any) -> None:
        """Send a command either via master or direct serial."""
        if self.master is not None:
            if self.device_id is None:
                raise RuntimeError("device_id not set for PlotClock")
            if cmd_name in self.commands:
                fmt = self.commands[cmd_name]
                cmd_str = fmt.format(*params)
            else:
                cmd_str = cmd_name
            prefix = f"P{self.device_id}."
            if not cmd_str.startswith(prefix):
                cmd_str = prefix + cmd_str
            self.master.send_command(cmd_str)
        else:
            super().send_command(cmd_name, *params)

    def _query_value(self, code: str, timeout: float = 1.5) -> str:
        """Send ``code`` and return the first response payload for this clock."""
        if self.master is None or self.device_id is None:
            raise RuntimeError("query requires master and device_id")

        prefix = f"P{self.device_id}."
        if not code.startswith(prefix):
            cmd = prefix + code
        else:
            cmd = code
        self.master.send_command(cmd)

        start = time.time()
        while time.time() - start < timeout:
            lines = self.master.get_lines()
            for line in lines:
                if line.startswith(f"P{self.device_id}:"):
                    payload = line.split(":", 1)[1]
                    return payload.rsplit(":", 1)[-1].strip()
            time.sleep(0.05)

        raise RuntimeError(f"Timed out waiting for response to {code}")

    # ──────────────────────────────────────────────────────────
    # Calibration public helper
    # ──────────────────────────────────────────────────────────
    def calibrate(self, detections: List[Union[Ball, ArucoMarker, ArucoHitter, ArucoManager]]) -> Optional[Dict[str,Any]]:
        """Feed per‑frame detections. Returns calibration dict once finished."""
        if self.calibration:  # already calibrated
            return self.calibration

        marker = next(
            (
                d
                for d in detections
                if isinstance(d, self.calibration_marker_cls)
                   and getattr(d, "id", None) == self.device_id  # ← NEW guard
            ),
            None,
        )

        now = time.time()

        # FSM --------------------------------------------------
        if self._cal_state == 0:
            x,y = self._mm_pts[0]
            # send a move command to the associated PlotClock. Prefix
            # ``P{device_id}.`` is added automatically by ``send_command`` so
            # we only issue the raw command here.
            self.send_command(f"p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 1
            return None

        if self._cal_state == 1 and now-self._last_cmd_t >= self._delay and marker:
            self._px_hits.append(marker.center)
            x,y = self._mm_pts[1]
            self.send_command(f"p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 2
            return None

        if self._cal_state == 2 and now-self._last_cmd_t >= self._delay and marker:
            self._px_hits.append(marker.center)
            x,y = self._mm_pts[2]
            self.send_command(f"p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 3
            return None

        if self._cal_state == 3 and now-self._last_cmd_t >= self._delay and marker:
            self._px_hits.append(marker.center)
            # compute basis -----------------------------------
            p1,p2,p3 = map(np.array,self._px_hits)
            (m1x,m1y),(m2x,m2y),(m3x,m3y) = self._mm_pts
            self._u_x = (p3-p2)/(m3x-m2x)
            self._u_y = (p1-p2)/(m1y-m2y)
            self._origin_px = p2 - self._u_x*m2x - self._u_y*m2y

            self.calibration = {
                'u_x': self._u_x,
                'u_y': self._u_y,
                'origin_px': tuple(self._origin_px.astype(int))
            }
            self._cal_state = 4  # done
            return self.calibration

        return None

    # ──────────────────────────────────────────────────────────
    # Conversion utilties
    # ──────────────────────────────────────────────────────────
    def _require_cal(self):
        if not self.calibration:
            raise RuntimeError('PlotClock is not calibrated yet')

    def pixel_to_mm(self, pixel_pt: Tuple[int,int]) -> Tuple[float,float]:
        self._require_cal()
        px = np.array(pixel_pt, dtype=float)
        ox,oy = self.calibration['origin_px']
        v = px - np.array([ox,oy])
        M = np.stack((self.calibration['u_x'], self.calibration['u_y']), axis=1)
        mm = np.linalg.inv(M).dot(v)
        return float(mm[0]), float(mm[1])

    def mm_to_pixel(self, mm_pt: Tuple[float, float]) -> Tuple[int, int]:
        self._require_cal()
        u_x = self.calibration['u_x']; u_y = self.calibration['u_y']
        ox, oy = self.calibration['origin_px']
        px = ox + u_x[0] * mm_pt[0] + u_y[0] * mm_pt[1]
        py = oy + u_x[1] * mm_pt[0] + u_y[1] * mm_pt[1]
        return int(px), int(py)

    # convenience ---------------------------------------------------------
    def find_mm(self, x_px: int, y_px: int) -> Tuple[float, float]:
        """Return (x_mm, y_mm) for a raw pixel coordinate."""
        return self.pixel_to_mm((x_px, y_px))


    def attack(self, frame_size: Tuple[int, int], target_mm: Tuple[float, float]):
        """Return a FixedTargetAttacker scenario for this clock."""
        from .scenarios import FixedTargetAttacker
        return FixedTargetAttacker(self, frame_size, target_mm)

    def defend(self, frame_size: Tuple[int, int]):
        """Return a BallReflector scenario for this clock."""
        from .scenarios import BallReflector
        return BallReflector(self, frame_size)

    def hit_standing_ball(self, target_mm: Tuple[float, float] | None = None):
        """Return a StandingBallHitter scenario for this clock."""
        from .scenarios import StandingBallHitter
        return StandingBallHitter(self, target_mm)

    def wait_position_mm(self) -> Tuple[float, float]:
        """Return a safe waiting (x,y) inside the workspace."""
        if self.WAIT_POS_MM is not None:
            return self.WAIT_POS_MM
        return (0.0, self.min_y + self.cal_margin_mm)

    def draw_working_area(
        self,
        frame: np.ndarray,
        *,
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> None:
        """Draw the reachable working space of this clock on ``frame``."""
        import cv2

        if not self.calibration:
            return

        r = self.workspace_radius
        d = self.workspace_dist
        max_x = self.max_x
        min_y = self.min_y

        num = 50
        xs_left = np.linspace(-max_x, 0, num)
        ys_left = np.sqrt(np.clip(r * r - (xs_left - d) ** 2, 0, None))
        xs_right = np.linspace(0, max_x, num)
        ys_right = np.sqrt(np.clip(r * r - (xs_right + d) ** 2, 0, None))

        pts_mm: List[Tuple[float, float]] = list(zip(xs_left, ys_left))
        pts_mm += list(zip(xs_right, ys_right))
        pts_mm.append((max_x, min_y))
        pts_mm.append((-max_x, min_y))

        pts_px = [self.mm_to_pixel(p) for p in pts_mm]
        poly = np.array(pts_px, dtype=np.int32)
        cv2.polylines(frame, [poly], True, color, thickness)

        if (
            self._origin_px is not None
            and self._u_x is not None
            and self._u_y is not None
        ):
            origin = tuple(int(v) for v in self._origin_px)
            axis_len = max(30, int(0.1 * min(frame.shape[:2])))
            end_x = (
                int(origin[0] + self._u_x[0] * axis_len),
                int(origin[1] + self._u_x[1] * axis_len),
            )
            end_y = (
                int(origin[0] + self._u_y[0] * axis_len),
                int(origin[1] + self._u_y[1] * axis_len),
            )
            cv2.arrowedLine(frame, origin, end_x, (255, 0, 0), 2, tipLength=0.2)
            cv2.arrowedLine(frame, origin, end_y, (0, 0, 255), 2, tipLength=0.2)

    def get_working_area_polygon(self) -> List[Tuple[int, int]]:
        """Return the pixel coordinates outlining the working area."""
        if not self.calibration:
            return []

        r = self.workspace_radius
        d = self.workspace_dist
        max_x = self.max_x
        min_y = self.min_y

        num = 50
        xs_left = np.linspace(-max_x, 0, num)
        ys_left = np.sqrt(np.clip(r * r - (xs_left - d) ** 2, 0, None))
        xs_right = np.linspace(0, max_x, num)
        ys_right = np.sqrt(np.clip(r * r - (xs_right + d) ** 2, 0, None))

        pts_mm: List[Tuple[float, float]] = list(zip(xs_left, ys_left))
        pts_mm += list(zip(xs_right, ys_right))
        pts_mm.append((max_x, min_y))
        pts_mm.append((-max_x, min_y))

        return [self.mm_to_pixel(p) for p in pts_mm]


class ArenaManager(PlotClock):
    """PlotClock variant representing the arena manager.

    If an :class:`Arena` instance is assigned via :meth:`set_arena`, the
    working area visualisation simply draws that arena polygon instead of the
    default semicircular reach of :class:`PlotClock`.
    """

    calibration_marker_cls = ArucoManager
    WAIT_POS_MM: Tuple[float, float] | None = (0.0, 250.0)
    SERVO_MM_DIST: float = 148.0

    def grip(self) -> None:
        """Close the manager's gripper."""
        self.send_command("p.grip()")

    def release(self) -> None:
        """Open the manager's gripper."""
        self.send_command("p.release()")

    def grip_smooth(self, end: int = 50, step: int = 1, delay: float = 0.02) -> None:
        """Smoothly close the gripper to ``end`` angle."""
        self.send_command(f"p.grip_smooth({end},{step},{delay})")

    def release_smooth(self, end: int = 180, step: int = 1, delay: float = 0.02) -> None:
        """Smoothly open the gripper to ``end`` angle."""
        self.send_command(f"p.release_smooth({end},{step},{delay})")

    def __init__(self, *args, arena: Optional[Arena] = None, coeffs_path: str | None = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.arena: Optional[Arena] = arena
        self._servo_px_dist: Optional[float] = None
        self._coeff_x: Optional[np.ndarray] = None
        self._coeff_y: Optional[np.ndarray] = None
        self._manager_center_px: Optional[Tuple[int, int]] = None
        self._pending_target_mm: Optional[Tuple[float, float]] = None
        self.relative_error_px: Optional[Tuple[float, float]] = None
        self.relative_error_mm: Optional[Tuple[float, float]] = None

        if coeffs_path:
            print(f"Loading coefficients from {coeffs_path}")
            self.load_correction(coeffs_path)

    # ------------------------------------------------------------------
    def record_manager_position(self, center_px: Tuple[int, int]) -> None:
        """Record the latest detected manager marker position."""

        self._manager_center_px = center_px

    # ------------------------------------------------------------------
    @staticmethod
    def _eval_poly(coeffs: np.ndarray, x: float, y: float) -> float:
        """Evaluate a quadratic polynomial."""
        return (
            coeffs[0]
            + coeffs[1] * x
            + coeffs[2] * y
            + coeffs[3] * x * x
            + coeffs[4] * x * y
            + coeffs[5] * y * y
        )

    def load_correction(self, path: str) -> None:
        """Load polynomial coefficients from ``path``."""
        import csv

        try:
            with open(path, newline="") as f:
                r = csv.reader(f)
                rows = [row for row in r]
        except OSError:
            return

        try:
            if len(rows) >= 4:
                self._coeff_x = np.array([float(v) for v in rows[1]], dtype=float)
                self._coeff_y = np.array([float(v) for v in rows[3]], dtype=float)
            elif len(rows) >= 2:
                self._coeff_x = np.array([float(v) for v in rows[0]], dtype=float)
                self._coeff_y = np.array([float(v) for v in rows[1]], dtype=float)
            print(f"Using coefficients {self._coeff_x} and {self._coeff_y}")
        except (ValueError, IndexError):
            self._coeff_x = None
            self._coeff_y = None


    def _apply_correction(self, x: float, y: float) -> Tuple[float, float]:
        """Return corrected coordinates if polynomial is loaded."""
        if self._coeff_x is not None and self._coeff_y is not None:
            cx = self._eval_poly(self._coeff_x, x, y)
            cy = self._eval_poly(self._coeff_y, x, y)
            return cx, cy
        return x, y

    # ------------------------------------------------------------------
    @staticmethod
    def _pt_in_poly(pt: Tuple[int, int], poly: List[Tuple[int, int]]) -> bool:
        """Return True if ``pt`` lies inside polygon ``poly``."""
        x, y = pt
        inside = False
        j = len(poly) - 1
        for i, (xi, yi) in enumerate(poly):
            xj, yj = poly[j]
            intersect = (
                (yi > y) != (yj > y)
                and x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi
            )
            if intersect:
                inside = not inside
            j = i
        return inside

    def set_arena(self, arena: Optional[Arena]) -> None:
        """Assign the current arena (or ``None`` to clear)."""
        self.arena = arena

    # ------------------------------------------------------------------
    def calibrate(
        self, detections: List[Union[Ball, ArucoMarker, ArucoHitter, ArucoManager]]
    ) -> Optional[Dict[str, Any]]:
        """Calibrate using two servo markers (ID 47) and manager marker (ID 0)."""

        if self.calibration:
            return self.calibration

        servos = [
            d
            for d in detections
            if isinstance(d, ArucoMarker) and getattr(d, "id", None) == 47
        ]
        mgr = next((d for d in detections if isinstance(d, ArucoManager)), None)

        if len(servos) >= 2 and mgr is not None:
            p1 = np.array(servos[0].center, dtype=float)
            p2 = np.array(servos[1].center, dtype=float)
            mid = (p1 + p2) / 2.0
            dx = -abs(p2 - p1)
            px_dist = float(np.linalg.norm(dx))
            if px_dist < 1e-6:
                return None
            scale = px_dist / self.SERVO_MM_DIST
            x_hat = dx / px_dist
            u_x = x_hat * scale
            y_hat = np.array([-x_hat[1], x_hat[0]])
            mgr_vec = np.array(mgr.center, dtype=float) - mid
            if np.dot(mgr_vec, y_hat) < 0:
                y_hat = -y_hat
            u_y = y_hat * scale

            self._u_x = u_x
            self._u_y = u_y
            self._origin_px = mid
            self._servo_px_dist = px_dist
            self.calibration = {
                "u_x": u_x,
                "u_y": u_y,
                "origin_px": (int(mid[0]), int(mid[1])),
                "servo_px_dist": px_dist,
            }
            return self.calibration

        return None

    def draw_working_area(
        self,
        frame: np.ndarray,
        *,
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 2,
    ) -> None:
        import cv2

        if self.arena is not None:
            corners = self.arena.get_arena_corners()
            if len(corners) >= 3:
                poly = np.array(corners, dtype=np.int32)
                cv2.polylines(frame, [poly], True, color, thickness)
                if (
                    self._origin_px is not None
                    and self._u_x is not None
                    and self._u_y is not None
                ):
                    origin = tuple(int(v) for v in self._origin_px)
                    axis_len = max(30, int(0.1 * min(frame.shape[:2])))
                    end_x = (
                        int(origin[0] + self._u_x[0] * axis_len),
                        int(origin[1] + self._u_x[1] * axis_len),
                    )
                    end_y = (
                        int(origin[0] + self._u_y[0] * axis_len),
                        int(origin[1] + self._u_y[1] * axis_len),
                    )
                    cv2.arrowedLine(frame, origin, end_x, (255, 0, 0), 2, tipLength=0.2)
                    cv2.arrowedLine(frame, origin, end_y, (0, 0, 255), 2, tipLength=0.2)
                return

        # Fall back to the regular PlotClock workspace
        super().draw_working_area(frame, color=color, thickness=thickness)

    def get_working_area_polygon(self) -> List[Tuple[int, int]]:
        """Return the polygon of the current arena or workspace."""
        if self.arena is not None:
            corners = self.arena.get_arena_corners()
            if len(corners) >= 3:
                return corners
            return []
        return super().get_working_area_polygon()

    # ------------------------------------------------------------------
    def update_manager_position(self, center_px: Tuple[int, int]) -> None:
        """Update the last detected manager marker position and apply feedback."""

        self._manager_center_px = center_px
        if (
            self._pending_target_mm is not None
            and self.calibration is not None
        ):
            tgt_mm = self._pending_target_mm
            tgt_px = self.mm_to_pixel(tgt_mm)
            dx_px = tgt_px[0] - center_px[0]
            dy_px = tgt_px[1] - center_px[1]
            self.relative_error_px = (float(dx_px), float(dy_px))
            cur_mm = self.pixel_to_mm(center_px)
            dx_mm = tgt_mm[0] - cur_mm[0]
            dy_mm = tgt_mm[1] - cur_mm[1]
            self.relative_error_mm = (float(dx_mm), float(dy_mm))

            # Send corrective relative move directly via base implementation
            PlotClock.send_command(self, f"p.setXYrel({dx_mm}, {dy_mm})")
            time.sleep(1.0)
            self._pending_target_mm = None

    # ------------------------------------------------------------------
    def setXY_updated_manager(self, x: float, y: float) -> None:
        """Move to ``(x, y)`` and trigger a background feedback update."""

        self.send_command(f"p.setXY({x}, {y})")

        def _worker() -> None:
            time.sleep(2.0)
            if self._manager_center_px is not None:
                self.update_manager_position(self._manager_center_px)

        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------------
    def send_command(self, cmd_name: str, *params: Any) -> None:
        """Intercept ``p.setXY`` to apply arena bounds and correction."""
        """Prevent out-of-bounds moves and track target for feedback."""
        target_mm: Optional[Tuple[float, float]] = None
        if isinstance(cmd_name, str) and cmd_name.strip().startswith("p.setXY"):
            import re

            m = re.match(r"p\.setXY\(([^,]+),([^\)]+)\)", cmd_name.replace(" ", ""))
            if m:
                try:
                    x = float(m.group(1))
                    y = float(m.group(2))
                    target_mm = (x, y)
                except ValueError:
                    target_mm = None

        if (
            target_mm is not None
            and self.arena is not None
            and self.calibration is not None
        ):
            if self.arena is not None and self.calibration:
                px, py = self.mm_to_pixel(target_mm)
                corners = self.arena.get_arena_corners()
                if len(corners) >= 3 and not self._pt_in_poly((px, py), corners):
                    return  # outside arena, ignore command

                x, y = self._apply_correction(x, y)
                cmd_name = f"p.setXY({x},{y})"

        super().send_command(cmd_name, *params)

        if target_mm is not None:
            self._pending_target_mm = target_mm

    # ------------------------------------------------------------------
    def move_object(self, obj: Union[Ball, Obstacle], target_x: float, target_y: float):
        """Return a MoveObject scenario for this arena manager."""
        from .scenarios import MoveObject

        return MoveObject(self, obj, (target_x, target_y))

    # backwards compatibility
    grab_and_release = move_object
