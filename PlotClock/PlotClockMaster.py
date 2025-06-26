"""Simple master controller for PlotClock slave.

This MicroPython script runs on a Raspberry Pi Pico that acts as the
``master`` device.  Commands received over the USB serial console are
forwarded verbatim to a slave Pico via UART0.  Any lines coming back
from the slave are echoed to the USB console.
"""

import machine
import sys
import utime
import select


UART_BAUD = 115200
uart = machine.UART(0, baudrate=UART_BAUD, tx=machine.Pin(0), rx=machine.Pin(1))

# ─────────────────── USB‑serial helpers ────────────────────────────────
_usb_poll = select.poll()
_usb_poll.register(sys.stdin, select.POLLIN)
_usb_buf = bytearray()


def _read_usb_line():
    """Return one complete line from USB or ``None`` if not ready."""
    global _usb_buf
    for fd, ev in _usb_poll.poll(0):
        if ev & select.POLLIN:
            ch = sys.stdin.read(1)
            if not ch or ch == "\r":
                continue
            if ch == "\n":
                try:
                    line = _usb_buf.decode()
                finally:
                    _usb_buf = bytearray()
                return line
            _usb_buf += ch.encode()
    return None


def _forward_uart_to_usb():
    """Echo any pending UART line to the USB console."""
    if uart.any():
        line = uart.readline()
        if line:
            try:
                text = line.decode().strip()
            except Exception:
                text = str(line)
            sys.stdout.write(text + "\n")
            if hasattr(sys.stdout, "flush"):
                sys.stdout.flush()


sys.stdout.write("[Master ready]\n")

while True:
    try:
        _forward_uart_to_usb()

        cmd = _read_usb_line()
        if cmd is not None:
            uart.write(cmd.encode() + b"\n")

        utime.sleep_ms(10)
    except KeyboardInterrupt:
        break

