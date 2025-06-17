import time
import cv2
import threading
from typing import Tuple, Optional


class Camera:
    """
    Encapsulates webcam capture using OpenCV in a background thread.
    Now with automatic FPS measurement.
    """
    def __init__(
        self,
        src: int = 0,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fourcc: Optional[str] = None,
    ):
        """Initialize a camera capture session."""
        self.src = src
        self.cap = cv2.VideoCapture(self.src)

        # Request a specific pixel format if provided (e.g. "MJPG" for MJPEG).
        if fourcc is not None:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # ── FPS detection ───────────────────────────────────
        fps_query = self.cap.get(cv2.CAP_PROP_FPS)
        self._fps: float = fps_query if fps_query and fps_query > 0 else 0.0
        self._fps_ready  = self._fps > 0

        # for runtime measurement when driver gives 0
        self._t0: float   = 0.0
        self._count: int  = 0
        self._MEASURE_N   = 60        # frames to measure

        # frame buffer
        self.grabbed: bool = False
        self.frame         = None
        self.lock          = threading.Lock()

        # thread control
        self.running = False
        self.thread: Optional[threading.Thread] = None

    # ───────────────────────────────────────────────────────
    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self) -> None:
        while self.running:
            grabbed, frame = self.cap.read()

            # runtime FPS estimation if needed
            if not self._fps_ready:
                if self._count == 0:
                    self._t0 = time.time()
                self._count += 1
                if self._count >= self._MEASURE_N:
                    t1 = time.time()
                    dt = max(1e-6, t1 - self._t0)
                    self._fps = self._count / dt
                    self._fps_ready = True

            with self.lock:
                self.grabbed = grabbed
                self.frame   = frame

    # ───────────────────────────────────────────────────────
    def read(self) -> Tuple[bool, Optional[any]]:
        with self.lock:
            if self.frame is None:
                return False, None
            return self.grabbed, self.frame.copy()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()
        self.cap.release()

    # ─── Accessors ─────────────────────────────────────────
    def get_resolution(self) -> Tuple[int, int]:
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def get_fps(self) -> float:
        """Return the measured (or driver-reported) FPS; 0.0 if not ready yet."""
        return self._fps if self._fps_ready else 0.0