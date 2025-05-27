import cv2
import threading
from typing import Tuple, Optional


class Camera:
    """
    Encapsulates webcam capture using OpenCV in a background thread.

    Usage:
        cam = Camera(src=0)
        cam.start()
        grabbed, frame = cam.read()
        cam.stop()
    """
    def __init__(self, src: int = 0, width: Optional[int] = None, height: Optional[int] = None):
        # Assign source before using
        self.src = src
        # Initialize video capture with given source (device index or path)
        self.cap = cv2.VideoCapture(self.src)

        # Only override resolution if specified; otherwise use native
        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.grabbed: bool = False
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """
        Start background frame capture.
        """
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self) -> None:
        """
        Continuously capture frames until stopped.
        """
        while self.running:
            grabbed, frame = self.cap.read()
            with self.lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self) -> Tuple[bool, Optional[any]]:
        """
        Retrieve the latest frame.

        Returns:
            grabbed (bool): Whether frame was successfully read.
            frame (numpy.ndarray or None): The captured frame.
        """
        with self.lock:
            if self.frame is None:
                return False, None
            return self.grabbed, self.frame.copy()

    def stop(self) -> None:
        """
        Stop background capture and release resources.
        """
        self.running = False
        if self.thread:
            self.thread.join()
        self.cap.release()

    def get_resolution(self) -> Tuple[int, int]:
        """
        Return the current capture resolution as (width, height) in pixels.
        """
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height
