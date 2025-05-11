"""
plotclock_client.py
Simple sender for PlotClock / vurucu5000 firmware.

Usage
-----
import plotclock_client as pc

pc.connect()                   # auto‑detect, or pc.connect("COM7") / pc.connect("/dev/ttyACM0")
pc.sendCommand("drawto 80 60") # always ends each command with '\n'
"""

import serial
import serial.tools.list_ports
import sys
import time

# ----------------------------------------------------------------------
_ser = None                     # global serial handle used by sendCommand()


def _find_pico_port() -> str | None:
    """
    Scan available ports and return the first one that looks like a Pico.
    Works on Windows, macOS and Linux.
    """
    for p in serial.tools.list_ports.comports():
        # Raspberry Pi Pico shows up with various VID:PID combos; most common:
        if (p.vid, p.pid) in {
            (0x2E8A, 0x0005),   # RP2040 USB serial
            (0x2E8A, 0x000A),   # CDC
        } or "Pico" in p.description:
            return p.device
    return None


def connect(port: str | None = None, baudrate: int = 115200, timeout: float = 0.2) -> None:
    """
    Open the serial connection.
    If *port* is None, try to auto‑detect the Pico’s port.
    Raises RuntimeError if it cannot connect.
    """
    global _ser
    if _ser and _ser.is_open:
        return                      # already connected

    if port is None:
        port = _find_pico_port()
        if port is None:
            raise RuntimeError("Could not auto‑detect Pico serial port; pass port name explicitly.")

    try:
        _ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)               # wait for Pico USB‑CDC to reset
        print(f"[plotclock_client] Connected to {port} @ {baudrate} baud")
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to open serial port {port}: {e}") from e


def sendCommand(cmd: str) -> None:
    """
    Send one line to the firmware (adds '\n' automatically).
    Call connect() once beforehand.
    """
    if _ser is None or not _ser.is_open:
        raise RuntimeError("Serial port not open – call connect() first.")

    if not cmd.endswith("\n"):
        cmd += "\n"
    _ser.write(cmd.encode("utf‑8"))
    _ser.flush()                    # make sure it goes out immediately


def close() -> None:
    """Close the serial port (optional)."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()
        print("[plotclock_client] Port closed")
        _ser = None


# ----------------------------------------------------------------------
if __name__ == "__main__":
    # quick CLI test: python plotclock_client.py "gohome"
    if len(sys.argv) > 1:
        connect()
        sendCommand(" ".join(sys.argv[1:]))
        close()