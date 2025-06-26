import threading
import sys
from typing import Optional, List
from collections import deque
import serial
import serial.tools.list_ports
import time

class MasterPico:
    """Simple manager for a master Pico handling multiple PlotClock slaves."""

    def __init__(self, port: Optional[str] = None, *, baudrate: int = 115200, timeout: float = 0.2) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._interactive = False
        self._lines: deque[str] = deque(maxlen=200)

    # ------------------------------------------------------------------
    def _find_port(self) -> Optional[str]:
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) in {(0x2E8A, 0x0005), (0x2E8A, 0x000A)} or "Pico" in p.description:
                return p.device
        return None

    # ------------------------------------------------------------------
    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return
        port = self.port or self._find_port()
        if not port:
            raise RuntimeError("Could not auto-detect serial port; specify port explicitly.")
        try:
            self._ser = serial.Serial(port, baudrate=self.baudrate, timeout=self.timeout)
            time.sleep(2)  # allow USB-CDC reset
            self._start_reader()
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}") from e

    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    # ------------------------------------------------------------------
    def _reader(self) -> None:
        while self._ser and self._ser.is_open:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                text = line.decode("utf-8", "replace").rstrip()
                self._lines.append(text)
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

    # ------------------------------------------------------------------
    def send_command(self, cmd: str) -> None:
        if not self.connected:
            raise RuntimeError("Serial port not open; call connect() first.")
        if not cmd.endswith("\n"):
            cmd += "\n"
        self._ser.write(cmd.encode("utf-8"))
        self._ser.flush()

    # ------------------------------------------------------------------
    def get_lines(self) -> List[str]:
        lines = list(self._lines)
        self._lines.clear()
        return lines

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
