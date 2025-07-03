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
from typing import Any, Dict, List, Optional, Union, Tuple

import cv2
import numpy as np

from .camera import Camera
from .trackers import BallTracker, DETECTION_SCALE
from .detectors import ArucoDetector, BallDetector
from .pipelines import RawImagePipeline, MaskedImagePipeline, AnnotatedImagePipeline
from .renderers import render_overlay, draw_line
from .models import Ball, ArucoMarker, ArucoHitter, ArucoManager
from .gadgets import PlotClock, ArenaManager
from .master_pico import MasterPico
from .scenarios import *
from .simple_api import CommandScenario, sort_plotclocks
from .scenario_loader import load_scenario, ScenarioLoadError


class GameAPI:
    """Encapsulates the processing pipeline and scenario management."""

    def __init__(self, coeffs_path: str | None = None) -> None:
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
        self._coeffs_path = coeffs_path
        self.pico_lock = threading.Lock()
        self.pico_connected = False

        self._current_scenario: Scenario | None = None
        self.scenario_enabled = False
        self.clock_scenarios: Dict[int, Scenario] = {}
        self.clock_modes: Dict[int, str] = {}
        self.preview_targets: Dict[int, Tuple[float, float]] = {}
        self.selected_obj: Optional[Tuple[str, str]] = None
        self._cam_started = False
        # scenario that could be loaded automatically after detection but is not
        # active by default
        self.default_scenario: Optional[str] = None

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
                if m.id not in self.plotclocks:
                    if isinstance(m, ArucoManager):
                        self.plotclocks[m.id] = ArenaManager(
                            device_id=m.id,
                            master=self.master_pico,
                            coeffs_path=self._coeffs_path,
                        )
                    elif isinstance(m, ArucoHitter):
                        self.plotclocks[m.id] = PlotClock(device_id=m.id, master=self.master_pico)

            # record ArenaManager marker positions
            for m in markers:
                if isinstance(m, ArucoManager):
                    clock = self.plotclocks.get(m.id)
                    if isinstance(clock, ArenaManager):
                        clock.record_manager_position(m.center)

        scenario_lines = []
        extra_pts: List[tuple] = []
        extra_labels: List[str] = []
        detections = balls + markers
        if self._current_scenario and self.scenario_enabled:
            self._current_scenario.update(detections)
            line = self._current_scenario.get_line_points()
            if line:
                scenario_lines.extend(line if isinstance(line, list) else [line])
            pts = self._current_scenario.get_extra_points()
            if pts:
                extra_pts.extend(pts if isinstance(pts, list) else [pts])
            labels = self._current_scenario.get_extra_labels()
            if labels:
                extra_labels.extend(labels if isinstance(labels, list) else [labels])
            if self._current_scenario.finished:
                self.stop_scenario()

        finished_ids = []
        for dev_id, sc in list(self.clock_scenarios.items()):
            sc.update(detections)
            line = sc.get_line_points()
            if line:
                scenario_lines.extend(line if isinstance(line, list) else [line])
            pts = sc.get_extra_points()
            if pts:
                extra_pts.extend(pts if isinstance(pts, list) else [pts])
            labels = sc.get_extra_labels()
            if labels:
                extra_labels.extend(labels if isinstance(labels, list) else [labels])
            if sc.finished:
                finished_ids.append(dev_id)

        for dev_id in finished_ids:
            self.stop_clock_mode(dev_id)
            self.clear_preview_target(dev_id)

        with self.lock:
            preview = list(self.preview_targets.items())
        for dev_id, tgt in preview:
            clock = self.plotclocks.get(dev_id)
            if isinstance(clock, ArenaManager) and clock.calibration:
                extra_pts.append(clock.mm_to_pixel(tgt))
                extra_labels.append("Target")

        annotated = render_overlay(
            frame,
            balls,
            markers,
            line_points=None,
            extra_points=extra_pts if extra_pts else None,
            extra_labels=extra_labels if extra_labels else None,
            highlight={"type": self.selected_obj[0], "id": self.selected_obj[1]} if self.selected_obj else None,
        )

        for p1, p2 in scenario_lines:
            draw_line(annotated, p1, p2, color=(255, 255, 255), thickness=2)

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

    # --- per PlotClock scenarios -------------------------------------
    def start_clock_mode(
        self,
        clock: PlotClock,
        mode: str,
        target_mm: Tuple[float, float] | None = None,
    ) -> None:
        if mode == "attack":
            tgt = target_mm if target_mm is not None else (100.0, 0.0)
            sc = clock.attack(self.frame_size, tgt)
        elif mode == "defend":
            sc = clock.defend(self.frame_size)
        elif mode == "hit_standing":
            sc = clock.hit_standing_ball(target_mm)
        elif mode == "move_object":
            raise ValueError("move_object requires parameters")
        else:
            raise ValueError("bad mode")
        sc.on_start()
        self.clock_scenarios[clock.device_id] = sc
        self.clock_modes[clock.device_id] = mode

    def stop_clock_mode(self, device_id: int) -> None:
        sc = self.clock_scenarios.pop(device_id, None)
        mode = self.clock_modes.pop(device_id, None)
        if sc:
            sc.on_stop()
        if mode == "move_object":
            self.set_selected_object(None)

    def start_move_object(
        self,
        manager: ArenaManager,
        obj: Union[Ball, Obstacle],
        target_mm: Tuple[float, float],
    ) -> None:
        sc = manager.move_object(obj, target_mm[0], target_mm[1])
        sc.on_start()
        self.clock_scenarios[manager.device_id] = sc
        self.clock_modes[manager.device_id] = "move_object"

    def set_preview_target(self, device_id: int, target_mm: Tuple[float, float]) -> None:
        with self.lock:
            self.preview_targets[device_id] = target_mm

    def clear_preview_target(self, device_id: int) -> None:
        with self.lock:
            self.preview_targets.pop(device_id, None)

    def set_selected_object(self, obj: Optional[Tuple[str, str]]) -> None:
        with self.lock:
            self.selected_obj = obj

    def process_message(self, message: Dict[str, Any]) -> None:
        dev_id = message.get("device_id")
        if dev_id is not None and dev_id in self.clock_scenarios:
            self.clock_scenarios[dev_id].process_message(message)
            return
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
            "selected_obj": self.selected_obj,
            "default_scenario": self.default_scenario,
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

        gadget_details = []
        for g in self.plotclocks.values():
            details = {
                "class": g.__class__.__name__,
                "calibrated": bool(getattr(g, "calibration", None)),
                "id": getattr(g, "device_id", None),
            }
            if hasattr(g, "_u_x") and g._u_x is not None:
                details["u_x"] = g._u_x.tolist()
            if hasattr(g, "_u_y") and g._u_y is not None:
                details["u_y"] = g._u_y.tolist()
            if hasattr(g, "_origin_px") and g._origin_px is not None:
                details["origin_px"] = [int(v) for v in g._origin_px]
            gadget_details.append(details)

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
                "active_modes": self.clock_modes,
            }
        )
        return info
