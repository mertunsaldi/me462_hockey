import threading
import time
from typing import Optional, Callable
import numpy as np

from camera import Camera
from detectors import BallDetector
from trackers import DETECTION_SCALE


class RawImagePipeline:
    """Capture raw frames from a camera in a background thread."""

    def __init__(self, camera: Camera):
        self.camera = camera
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            grabbed, frame = self.camera.read()
            if not grabbed:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()

    def get_raw_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.frame is None else self.frame.copy()


class MaskedImagePipeline:
    """Produce masked frames from a raw image pipeline."""

    def __init__(self, raw_pipe: RawImagePipeline, scale: float = DETECTION_SCALE):
        self.raw_pipe = raw_pipe
        self.scale = scale
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            frame = self.raw_pipe.get_raw_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            mask = BallDetector.get_mask(frame, scale=self.scale)
            with self.lock:
                self.frame = mask

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()

    def get_masked_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.frame is None else self.frame.copy()


class AnnotatedImagePipeline:
    """Generate annotated frames using a provided processor function."""

    def __init__(
        self,
        raw_pipe: RawImagePipeline,
        processor: Callable[[np.ndarray], np.ndarray],
    ):
        self.raw_pipe = raw_pipe
        self.processor = processor
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            frame = self.raw_pipe.get_raw_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            annotated = self.processor(frame)
            with self.lock:
                self.frame = annotated

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()

    def get_annotated_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.frame is None else self.frame.copy()
