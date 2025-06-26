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
from .models import Ball, ArucoMarker, ArucoHitter, ArucoManager
from .gadgets import PlotClock
from .master_pico import MasterPico
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

        self.master_pico = MasterPico(port=None, baudrate=115200, timeout=0.2)
        self.plotclocks: Dict[int, PlotClock] = {}
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

        if self.pico_connected:
            for m in markers:
                if isinstance(m, (ArucoHitter, ArucoManager)) and m.id not in self.plotclocks:
                    self.plotclocks[m.id] = PlotClock(device_id=m.id, master=self.master_pico)

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
            self.master_pico.connect()
            self.pico_connected = True

    def send_cmd(self, cmd: str) -> None:
        if not self.pico_connected:
            raise RuntimeError("not connected")
        with self.pico_lock:
            self.master_pico.send_command(cmd)

    def read_pico_lines(self) -> List[str]:
        with self.pico_lock:
            return self.master_pico.get_lines()

    # ------------------------------------------------------------------
    def load_commands(self, commands: List[Dict[str, Any]]) -> None:
        ordered = sort_plotclocks(list(self.plotclocks.values()))
        self._current_scenario = CommandScenario(ordered, self.frame_size, commands)
        self.scenario_enabled = False

    def load_scenario(self, path: str) -> None:
        if not self.plotclocks:
            raise RuntimeError("no plotclock available")
        first_clock = list(self.plotclocks.values())[0]
        self._current_scenario = load_scenario(path, first_clock, self.frame_size)
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
    def debug_info(self) -> Dict[str, Any]:
        """Extended diagnostics about the current system state."""

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

        scenario_name = (
            self._current_scenario.__class__.__name__
            if self._current_scenario
            else None
        )

        info: Dict[str, Any] = {
            "num_balls": len(balls),
            "ball_ids": [b.id for b in balls],
            "speeds": speeds,
            "ball_details": ball_details,
            "num_markers": len(markers),
            "marker_ids": [m.id for m in markers],
            "marker_centers": [m.center for m in markers],
            "scenario_loaded": self._current_scenario is not None,
            "scenario_running": self._current_scenario is not None
            and self.scenario_enabled,
            "scenario_name": scenario_name,
        }

        marker_details = [
            {
                "id": m.id,
                "center": m.center,
                "corners": m.corners,
                "type": m.__class__.__name__,
            }
            for m in markers
        ]

        gadget_details = [
            {
                "class": g.__class__.__name__,
                "calibrated": bool(getattr(g, "calibration", None)),
            }
            for g in self.plotclocks.values()
        ]

        image_params = {
            "circularity": BallDetector.CIRCULARITY_THRESHOLD,
            "area_ratio": BallDetector.AREA_RATIO_THRESHOLD,
            "solidity": BallDetector.SOLIDITY_THRESHOLD,
            "edge_density": BallDetector.EDGE_DENSITY_THRESHOLD,
            "blur_kernel": BallDetector.BLUR_KERNEL,
            "blur_sigma": BallDetector.BLUR_SIGMA,
            "hsv_lower": BallDetector.HSV_LOWER.tolist(),
            "hsv_upper": BallDetector.HSV_UPPER.tolist(),
            "min_radius": BallDetector.MIN_RADIUS,
            "max_radius": BallDetector.MAX_RADIUS,
            "detection_scale": DETECTION_SCALE,
        }

        info.update(
            {
                "markers": marker_details,
                "gadgets": gadget_details,
                "image_params": image_params,
                "camera_source": self.camera.src,
                "frame_size": self.frame_size,
            }
        )
        return info
