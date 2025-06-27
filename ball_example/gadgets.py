import threading
import sys
from typing import Optional, Any, Dict, List, Tuple, Union
import serial
import serial.tools.list_ports
import time
import math
import numpy as np
from .models import Ball, ArucoMarker, ArucoHitter, ArucoManager


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

    def __init__(
        self,
        *,
        device_id: Optional[int] = None,
        master: Optional[MasterPico] = None,
    ):
        super().__init__(None, baudrate=115200, timeout=20)
        self.device_id = device_id
        self.master = master
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
        self.links_fetched = False
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
            min_x = -max_x
            max_y = math.sqrt(max(0.0, length * length - dist * dist))
            self.x_range = (min_x, max_x)
            self.y_range = (min_y, max_y)
        else:
            self.max_x = 80.0
            self.min_y = 10.0
            self.workspace_radius = 120.0
            self.workspace_dist = 40.0
            self.x_range = (-self.max_x, self.max_x)
            self.y_range = (self.min_y, math.sqrt(max(0.0, self.workspace_radius ** 2 - self.workspace_dist ** 2)))

        self.links_fetched = True

        # Calibration state --------------------------------------------------
        self._cal_state: int = 0          # 0=idle,1..n=fsm
        span_x = 2 * self.max_x
        span_y = self.y_range[1] - self.min_y
        self._axis_len = 0.5 * min(span_x, span_y)
        base_x = -self._axis_len / 2
        base_y = self.min_y
        self._mm_pts = [
            (base_x, base_y + self._axis_len),
            (base_x, base_y),
            (base_x + self._axis_len, base_y),
        ]
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

    def _query_value(self, code: str, timeout: float = 1.0) -> str:
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
            (d for d in detections if isinstance(d, self.calibration_marker_cls)),
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
        ys_left = np.sqrt(np.clip(r * r - (xs_left + d) ** 2, 0, None))
        xs_right = np.linspace(0, max_x, num)
        ys_right = np.sqrt(np.clip(r * r - (xs_right - d) ** 2, 0, None))

        pts_mm: List[Tuple[float, float]] = list(zip(xs_left, ys_left))
        pts_mm += list(zip(xs_right, ys_right))
        pts_mm.append((max_x, min_y))
        pts_mm.append((-max_x, min_y))

        pts_px = [self.mm_to_pixel(p) for p in pts_mm]
        poly = np.array(pts_px, dtype=np.int32)
        cv2.polylines(frame, [poly], True, color, thickness)


class ArenaManager(PlotClock):
    """Placeholder class for arena manager devices."""

    calibration_marker_cls = ArucoManager
