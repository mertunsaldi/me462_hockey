"""
plotclock_client.py
Interactive sender + live feedback for PlotClock / vurucu5000 firmware.
"""

import serial, serial.tools.list_ports
import sys, time, threading

# ----------------------------------------------------------------------
_ser = None                 # global serial handle
_reader_thread = None       # background line‑reader
_interactive_mode = False   # set True while REPL is running


def _find_pico_port() -> str | None:
    """Return the first serial port that looks like a Pico."""
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) in {(0x2E8A, 0x0005), (0x2E8A, 0x000A)} or "Pico" in p.description:
            return p.device
    return None


# ──────────────────── Reader Thread ────────────────────────────────────
def _reader():
    """Continuously print every line arriving from Pico, then refresh prompt."""
    while _ser and _ser.is_open:
        try:
            line = _ser.readline()                # blocks up to timeout
            if not line:
                continue
            text = line.decode("utf-8", "replace").rstrip()

            # Move to new line, print text, then re‑show prompt if needed
            sys.stdout.write("\r\n" + text + "\n")
            if _interactive_mode:
                sys.stdout.write("> ")
            sys.stdout.flush()
        except (serial.SerialException, OSError):
            break
        except UnicodeDecodeError:
            continue    # skip bad bytes


def _start_reader():
    global _reader_thread
    if _reader_thread is None or not _reader_thread.is_alive():
        _reader_thread = threading.Thread(target=_reader, daemon=True)
        _reader_thread.start()


# ──────────────────── Public API ────────────────────────────────────────
def connect(port: str | None = None, *, baudrate: int = 115200, timeout: float = 0.2) -> None:
    """Open the serial connection (auto‑detect if *port* is None)."""
    global _ser
    if _ser and _ser.is_open:
        return
    if port is None:
        port = _find_pico_port()
        if port is None:
            raise RuntimeError("Could not auto‑detect Pico serial port; pass port name explicitly.")
    try:
        _ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)          # Pico resets USB‑CDC on open
        print(f"[plotclock_client] Connected to {port} @ {baudrate} baud")
        _start_reader()        # begin listening for feedback
    except serial.SerialException as e:
        raise RuntimeError(f"Failed to open serial port {port}: {e}") from e


def sendCommand(cmd: str) -> None:
    """Send one line to Pico (adds '\\n').  Call connect() first."""
    if _ser is None or not _ser.is_open:
        raise RuntimeError("Serial port not open – call connect() first.")
    if not cmd.endswith("\n"):
        cmd += "\n"
    _ser.write(cmd.encode("utf-8"))
    _ser.flush()


def close() -> None:
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()
        print("[plotclock_client] Port closed")
        _ser = None


# ──────────────────── Interactive REPL ─────────────────────────────────
def _interactive():
    """Simple REPL to send commands typed in the terminal."""
    global _interactive_mode
    _interactive_mode = True
    print("Type commands for the PlotClock.  Ctrl‑C or 'exit' to quit.")
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:          # Ctrl‑D
                break
            if not line:
                continue
            if line.lower() in {"exit", "quit"}:
                break
            try:
                sendCommand(line)
            except Exception as e:
                print("ERR:", e)
    except KeyboardInterrupt:
        pass
    finally:
        _interactive_mode = False
        close()


# ──────────────────── CLI entry point ──────────────────────────────────
if __name__ == "__main__":
    # argv[1] may contain explicit port name
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    connect(port_arg)

    # if extra args after port were provided, send them as one command then exit
    if len(sys.argv) > (2 if port_arg else 1):
        sendCommand(" ".join(sys.argv[2 if port_arg else 1:]))
        time.sleep(0.1)   # let Pico echo before closing
        close()
    else:
        _interactive()