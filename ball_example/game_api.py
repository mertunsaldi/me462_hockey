"""Core game logic separate from the Flask GUI.

The :class:`GameAPI` object owns the camera, detection pipelines and the
currently active scenario.  GUI frontends can obtain annotated frames
from it and invoke high level control methods.  The existing Flask
server uses this class but any other interface can do the same.
"""

from __future__ import annotations

import threading
import time
import math
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from .camera import Camera
from .trackers import BallTracker, DETECTION_SCALE
from .detectors import ArucoDetector, BallDetector
from .pipelines import RawImagePipeline, MaskedImagePipeline, AnnotatedImagePipeline
from .renderers import render_overlay, draw_line
from .models import Ball, ArucoMarker
from .gadgets import PlotClock
from .scenarios import *
from .simple_api import CommandScenario, sort_plotclocks
from .scenario_loader import load_scenario, ScenarioLoadError


class GameAPI:
    """Encapsulates the processing pipeline and scenario management."""

    def __init__(self) -> None:
        self.camera = Camera(src=2, width=1280, height=720, fourcc="MJPG")
        self.frame_size = self.camera.get_resolution()
        self.tracker_mgr = BallTracker()
        self.raw_pipe = RawImagePipeline(self.camera)
        self.mask_pipe = MaskedImagePipeline(self.raw_pipe, scale=DETECTION_SCALE)
        self.annotated_pipe: Optional[AnnotatedImagePipeline] = None

        self.balls: List[Ball] = []
        self.arucos: List[ArucoMarker] = []
        self.lock = threading.Lock()

        self.plotclock = PlotClock(port=None, baudrate=115200, timeout=0.2)
        self.plotclocks = [self.plotclock]
        self.pico_lock = threading.Lock()
        self.pico_connected = False

        self._current_scenario: Scenario | None = None
        self.scenario_enabled = False
        self._cam_started = False

    # ------------------------------------------------------------------
    def set_cam_source(
        self,
        src: int | str,
        *,
        width: int | None = None,
        height: int | None = None,
        fourcc: str | None = "MJPG",
    ) -> None:
        """Switch to a different camera source and restart pipelines if running."""

        was_running = self._cam_started

        if self.annotated_pipe:
            self.annotated_pipe.stop()
        self.raw_pipe.stop()
        self.mask_pipe.stop()
        self.camera.stop()

        self.camera = Camera(src=src, width=width, height=height, fourcc=fourcc)
        self.frame_size = self.camera.get_resolution()
        self.raw_pipe = RawImagePipeline(self.camera)
        self.mask_pipe = MaskedImagePipeline(self.raw_pipe, scale=DETECTION_SCALE)
        self.annotated_pipe = AnnotatedImagePipeline(self.raw_pipe, self._process_annotated)
        self._cam_started = False

        if was_running:
            self.start()

    # ------------------------------------------------------------------
    def start(self) -> None:
        if not self._cam_started and not self.camera.running:
            self.camera.start()
            self.raw_pipe.start()
            self.mask_pipe.start()
            if self.annotated_pipe is None:
                self.annotated_pipe = AnnotatedImagePipeline(
                    self.raw_pipe, self._process_annotated
                )
            self.annotated_pipe.start()
            self._cam_started = True

    # ------------------------------------------------------------------
    def _process_annotated(self, frame: np.ndarray) -> np.ndarray:
        mask = self.mask_pipe.get_masked_frame()
        balls = self.tracker_mgr.update(frame, mask=mask)
        markers = ArucoDetector.detect(frame)

        with self.lock:
            self.balls = balls
            self.arucos = markers

        scenario_line = None
        extra_pts = None
        extra_labels = None
        if self._current_scenario and self.scenario_enabled:
            detections = balls + markers
            self._current_scenario.update(detections)
            scenario_line = self._current_scenario.get_line_points()
            extra_pts = self._current_scenario.get_extra_points()
            extra_labels = self._current_scenario.get_extra_labels()

        annotated = render_overlay(
            frame,
            balls,
            markers,
            line_points=None,
            extra_points=extra_pts,
            extra_labels=extra_labels,
        )

        if self._current_scenario and self.scenario_enabled and scenario_line:
            lines = scenario_line if isinstance(scenario_line, list) else [scenario_line]
            for idx, (p1, p2) in enumerate(lines):
                if len(lines) == 2:
                    color = (255, 0, 0) if idx == 0 else (0, 0, 255)
                else:
                    color = (255, 255, 255)
                draw_line(annotated, p1, p2, color=color, thickness=2)

        return annotated

    # ------------------------------------------------------------------
    def get_annotated_frame(self) -> Optional[np.ndarray]:
        if self.annotated_pipe is None:
            return None
        return self.annotated_pipe.get_annotated_frame()

    def get_processed_frame(self) -> Optional[np.ndarray]:
        return self.mask_pipe.get_masked_frame()

    # ------------------------------------------------------------------
    def connect_pico(self) -> None:
        with self.pico_lock:
            if self.pico_connected:
                return
            self.plotclock.start_comms()
            self.plotclock.send_command("mode", 4)
            self.pico_connected = True

    def send_cmd(self, cmd: str) -> None:
        if not self.pico_connected:
            raise RuntimeError("not connected")
        with self.pico_lock:
            self.plotclock.send_command(cmd)

    # ------------------------------------------------------------------
    def load_commands(self, commands: List[Dict[str, Any]]) -> None:
        ordered = sort_plotclocks(self.plotclocks)
        self._current_scenario = CommandScenario(ordered, self.frame_size, commands)
        self.scenario_enabled = False

    def load_scenario(self, path: str) -> None:
        self._current_scenario = load_scenario(path, self.plotclock, self.frame_size)
        self.scenario_enabled = False

    def start_scenario(self) -> None:
        if self._current_scenario is None:
            raise RuntimeError("no scenario loaded")
        if not self.scenario_enabled:
            self._current_scenario.on_start()
            self.scenario_enabled = True

    def stop_scenario(self) -> None:
        if self._current_scenario and self.scenario_enabled:
            self._current_scenario.on_stop()
            self.scenario_enabled = False

    def process_message(self, message: Dict[str, Any]) -> None:
        if self._current_scenario:
            self._current_scenario.process_message(message)

    # ------------------------------------------------------------------
    def stats(self) -> Dict[str, Any]:
        with self.lock:
            balls = list(self.balls)
            markers = list(self.arucos)
        speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]
        ball_details = [
            {
                "id": b.id,
                "center": b.center,
                "radius": b.radius,
                "velocity": tuple(map(float, b.velocity)),
            }
            for b in balls
        ]
        scenario_name = self._current_scenario.__class__.__name__ if self._current_scenario else None
        return {
            "num_balls": len(balls),
            "ball_ids": [b.id for b in balls],
            "speeds": speeds,
            "ball_details": ball_details,
            "num_markers": len(markers),
            "marker_ids": [m.id for m in markers],
            "marker_centers": [m.center for m in markers],
            "scenario_loaded": self._current_scenario is not None,
            "scenario_running": self._current_scenario is not None and self.scenario_enabled,
            "scenario_name": scenario_name,
        }
