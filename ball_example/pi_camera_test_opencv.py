import cv2
import threading
import time
from collections import deque
from typing import Optional, Tuple

# ─── Optional Picamera2 import ──────────────────────────────────────────────
try:
    from picamera2 import Picamera2
    _PICAMERA2_AVAILABLE = True
except ImportError:
    _PICAMERA2_AVAILABLE = False


# ────────────────────────────────────────────────────────────────────────────
# USB / V4L camera (unchanged logic, just renamed)                            │
# ────────────────────────────────────────────────────────────────────────────
class USBCamera:
    def __init__(
        self,
        src: int = 0,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
    ):
        self.cap = cv2.VideoCapture(src)
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        self._grabbed = False
        self._frame   = None
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = None

    # public API -------------------------------------------------------------
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def read(self):
        with self._lock:
            if self._frame is None:
                return False, None
            return self._grabbed, self._frame.copy()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.cap.release()

    # private ----------------------------------------------------------------
    def _update(self):
        while self._running:
            grabbed, frame = self.cap.read()
            with self._lock:
                self._grabbed = grabbed
                self._frame   = frame


# ────────────────────────────────────────────────────────────────────────────
# Raspberry Pi Camera (dual-stream, keeps only newest frame)                  │
# ────────────────────────────────────────────────────────────────────────────
class PicameraCamera:
    """
    Full-resolution capture with Picamera2.
    • latest() gives you the newest BGR frame without copy (or None)
    • start()/stop() & read() mirror USBCamera for compatibility
    """
    def __init__(
        self,
        main_size:  Tuple[int, int] = (1280, 720),
        fmt: str = "RGB888",
        fps: int = 30,
    ):
        if not _PICAMERA2_AVAILABLE:
            raise RuntimeError("Picamera2 library not found")

        self.picam2 = Picamera2()
        self.config = self.picam2.create_video_configuration(
            main   = {"size": main_size, "format": fmt},
            lores  = None,
            display=None,
        )

        if fps:
            frame_len = int(1e6 / fps)
            self.config["controls"]["FrameDurationLimits"] = (frame_len, frame_len)

        self.picam2.configure(self.config)

        self._running     = False
        self._thread      = None
        self._latest_full = deque(maxlen=1)   # newest frame only

    # ───────── public API ───────────────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self.picam2.start()
        time.sleep(0.7)                         # ISP warm-up
        self._running = True
        self._thread  = threading.Thread(target=self._grab, daemon=True)
        self._thread.start()

    def read(self):
        """Compatibility wrapper → returns (True, copy_of_latest)"""
        frame = self.latest()
        return (frame is not None), None if frame is None else frame.copy()

    def latest(self):
        """Get newest frame without copying (may return None)."""
        return self._latest_full[-1] if self._latest_full else None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.picam2.stop()

    # ───────── grab loop ────────────────────────────────────────────────────
    def _grab(self):
        convert = cv2.COLOR_BGRA2BGR
        while self._running:
            arr = self.picam2.capture_array("main")
            self._latest_full.append(cv2.cvtColor(arr, convert))