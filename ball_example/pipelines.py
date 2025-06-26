import threading
import time
from typing import Optional, Callable
import numpy as np

from .camera import Camera
from .detectors import compute_color_mask
from .trackers import DETECTION_SCALE


class BasePipeline:
    """Common thread management and frame storage for pipelines."""

    def __init__(self) -> None:
        self.frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------
    def _run(self) -> None:
        while self.running:
            frame = self.process_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame

    # ------------------------------------------------------------------
    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join()

    # ------------------------------------------------------------------
    def process_frame(self) -> Optional[np.ndarray]:
        """Produce the next frame for the pipeline."""
        raise NotImplementedError

    # ------------------------------------------------------------------
    def get_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.frame is None else self.frame.copy()


class RawImagePipeline(BasePipeline):
    """Capture raw frames from a camera in a background thread."""

    def __init__(self, camera: Camera):
        super().__init__()
        self.camera = camera

    def process_frame(self) -> Optional[np.ndarray]:
        grabbed, frame = self.camera.read()
        if not grabbed:
            return None
        return frame

    def get_raw_frame(self) -> Optional[np.ndarray]:
        return self.get_frame()


class MaskedImagePipeline(BasePipeline):
    """Produce masked frames from a raw image pipeline."""

    def __init__(self, raw_pipe: RawImagePipeline, scale: float = DETECTION_SCALE):
        super().__init__()
        self.raw_pipe = raw_pipe
        self.scale = scale

    def process_frame(self) -> Optional[np.ndarray]:
        frame = self.raw_pipe.get_raw_frame()
        if frame is None:
            return None
        mask = compute_color_mask(frame, scale=self.scale)
        return mask

    def get_masked_frame(self) -> Optional[np.ndarray]:
        return self.get_frame()


class AnnotatedImagePipeline(BasePipeline):
    """Generate annotated frames using a provided processor function."""

    def __init__(
        self,
        raw_pipe: RawImagePipeline,
        processor: Callable[[np.ndarray], np.ndarray],
    ):
        super().__init__()
        self.raw_pipe = raw_pipe
        self.processor = processor

    def process_frame(self) -> Optional[np.ndarray]:
        frame = self.raw_pipe.get_raw_frame()
        if frame is None:
            return None
        annotated = self.processor(frame)
        return annotated

    def get_annotated_frame(self) -> Optional[np.ndarray]:
        return self.get_frame()
