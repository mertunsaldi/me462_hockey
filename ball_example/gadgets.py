import threading
import sys
from typing import Optional, Any, Dict, List, Tuple, Union
import serial
import serial.tools.list_ports
import time
import numpy as np
from models import Ball, ArucoMarker, ArucoHitter


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
    def __init__(self, port: Optional[str] = None, *, baudrate: int = 115200, timeout: float = 20):
        super().__init__(port, baudrate=baudrate, timeout=timeout)
        self.commands = {
        #istersek buraya commandlar ekleyebiliriz
        "mode": "mode {0}",    
        }
        # self.x_range = (0, 70)
        # self.y_range = (10, 55)
        self.x_range = (0, 80)
        self.y_range = (10, 55)

        # Calibration state --------------------------------------------------
        self._cal_state: int = 0          # 0=idle,1..n=fsm
        self._mm_pts: List[Tuple[float,float]] = [(100,300),(100,200),(200,200)]
        self._px_hits: List[Tuple[int,int]] = []
        self._last_cmd_t: float = 0.0
        self._delay: float = 2.0          # seconds between moves
        self._axis_len: float = 100.0     # mm for drawing axes
        # Results
        self.calibration: Optional[Dict[str,Any]] = None  # u_x,u_y,origin
        self._u_x: Optional[np.ndarray] = None
        self._u_y: Optional[np.ndarray] = None
        self._origin_px: Optional[np.ndarray] = None

    # ──────────────────────────────────────────────────────────
    # Calibration public helper
    # ──────────────────────────────────────────────────────────
    def calibrate(self, detections: List[Union[Ball,ArucoMarker,ArucoHitter]]) -> Optional[Dict[str,Any]]:
        """Feed per‑frame detections. Returns calibration dict once finished."""
        if self.calibration:  # already calibrated
            return self.calibration

        hitter = next((d for d in detections if isinstance(d,ArucoHitter)), None)
        now = time.time()

        # FSM --------------------------------------------------
        if self._cal_state == 0:
            x,y = self._mm_pts[0]
            self.send_command(f"P1.p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 1
            return None

        if self._cal_state == 1 and now-self._last_cmd_t >= self._delay and hitter:
            self._px_hits.append(hitter.center)
            x,y = self._mm_pts[1]
            self.send_command(f"P1.p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 2
            return None

        if self._cal_state == 2 and now-self._last_cmd_t >= self._delay and hitter:
            self._px_hits.append(hitter.center)
            x,y = self._mm_pts[2]
            self.send_command(f"P1.p.setXY({x}, {y})")
            self._last_cmd_t = now
            self._cal_state = 3
            return None

        if self._cal_state == 3 and now-self._last_cmd_t >= self._delay and hitter:
            self._px_hits.append(hitter.center)
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

