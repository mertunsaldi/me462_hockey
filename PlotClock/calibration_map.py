# -*- coding: utf-8 -*-
"""Calibration map generator for the ArenaManager.

This script moves the ArenaManager in a uniform grid and records where
its ArUco marker actually appears in the camera frame. The result is
saved as ``calibration_map.csv`` inside the PlotClock folder.

The pixel coordinates are measured relative to the average centre of
all detected 6x6 ArUco markers with id 47 which defines the origin.

While running a Tkinter window shows the live camera feed annotated
with the origin, marker centres and every measured grid point.
"""

from __future__ import annotations

import csv
import math
import time
from typing import List, Tuple

import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

from ball_example.game_api import GameAPI
from ball_example.models import ArucoMarker, ArucoManager
from ball_example.gadgets import ArenaManager
from ball_example.high_level import calibrate_clocks


NUM_POINTS = 50
MOVE_DELAY = 2.0  # seconds to wait after each move
CSV_NAME = "calibration_map.csv"


def _find_origin(markers: List[ArucoMarker]) -> Tuple[int, int] | None:
    pts = [m.center for m in markers if m.id == 47]
    if not pts:
        return None
    x = int(sum(p[0] for p in pts) / len(pts))
    y = int(sum(p[1] for p in pts) / len(pts))
    return x, y


def main() -> None:
    api = GameAPI()
    api.set_cam_source(0)
    api.start()
    api.connect_pico()

    origin: Tuple[int, int] | None = None
    manager: ArenaManager | None = None

    print("Waiting for ArenaManager and origin markers…")
    start = time.time()
    while time.time() - start < 10:
        time.sleep(0.1)
        with api.lock:
            markers = list(api.arucos)
            manager = next(
                (c for c in api.plotclocks.values() if isinstance(c, ArenaManager)),
                None,
            )
        if origin is None:
            origin = _find_origin(markers)
        if manager and origin:
            break
    if not manager or not origin:
        print("Required markers not found. Aborting.")
        return

    def _get_dets():
        with api.lock:
            return api.balls + api.arucos

    print("Calibrating ArenaManager…")
    calibrate_clocks([manager], _get_dets)

    # generate grid in working area
    nx = int(math.ceil(math.sqrt(NUM_POINTS)))
    ny = nx
    xs = np.linspace(manager.x_range[0], manager.x_range[1], nx)
    ys = np.linspace(manager.y_range[0], manager.y_range[1], ny)
    grid: List[Tuple[float, float]] = []
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            grid.append((float(x), float(y)))
            if len(grid) >= NUM_POINTS:
                break
        if len(grid) >= NUM_POINTS:
            break

    results: List[Tuple[Tuple[float, float], Tuple[int, int]]] = []

    # Tkinter setup --------------------------------------------------------
    root = tk.Tk()
    root.title("ArenaManager Calibration Map")
    label = tk.Label(root)
    label.pack()
    start_btn = tk.Button(root, text="Start")
    start_btn.pack()

    current_pt: Tuple[float, float] | None = None

    def update_frame() -> None:
        frame = api.get_annotated_frame()
        if frame is not None:
            with api.lock:
                markers = list(api.arucos)
            for m in markers:
                if m.id == 47:
                    cv2.circle(frame, m.center, 5, (0, 255, 0), -1)
            if manager.calibration:
                manager.draw_working_area(frame, color=(0, 255, 255), thickness=1)
            cv2.circle(frame, origin, 6, (0, 0, 255), 2)
            if manager.calibration:
                for p in grid:
                    px = manager.mm_to_pixel(p)
                    cv2.circle(frame, px, 3, (255, 255, 0), 1)
            for _, px in results:
                cv2.circle(frame, (px[0] + origin[0], px[1] + origin[1]), 4, (255, 0, 0), -1)
            if current_pt and manager.calibration:
                px = manager.mm_to_pixel(current_pt)
                cv2.circle(frame, px, 6, (0, 128, 255), 2)
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            im_tk = ImageTk.PhotoImage(im_pil)
            label.configure(image=im_tk)
            label.image = im_tk
        root.after(30, update_frame)

    idx = 0

    def record_position() -> None:
        nonlocal idx, current_pt
        with api.lock:
            markers = list(api.arucos)
        m = next((m for m in markers if isinstance(m, ArucoManager)), None)
        if m is not None:
            dx = m.center[0] - origin[0]
            dy = m.center[1] - origin[1]
            results.append((grid[idx], (dx, dy)))
            idx += 1
        current_pt = None
        if idx < len(grid):
            root.after(100, move_next)
        else:
            with open(CSV_NAME, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["cmd_x", "cmd_y", "px_x", "px_y"])
                for cmd, px in results:
                    writer.writerow([cmd[0], cmd[1], px[0], px[1]])
            print(f"Calibration map saved to {CSV_NAME}")

    def move_next() -> None:
        nonlocal current_pt
        current_pt = grid[idx]
        x, y = current_pt
        manager.send_command(f"p.setXY({x}, {y})")
        root.after(int(MOVE_DELAY * 1000), record_position)

    def start() -> None:
        start_btn.config(state=tk.DISABLED)
        root.after(0, move_next)

    start_btn.config(command=start)

    root.after(0, update_frame)
    root.mainloop()


if __name__ == "__main__":
    main()
