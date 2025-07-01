import time
import csv
import math
from typing import List, Tuple

import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

from ball_example.game_api import GameAPI
from ball_example.models import ArucoWall, Arena, ArucoMarker, ArucoManager
from ball_example.gadgets import ArenaManager
from high_level import calibrate_clocks, draw_arena


NUM_POINTS = 50


def compute_servo_origin(markers: List[ArucoMarker]) -> Tuple[int, int]:
    servos = [m for m in markers if isinstance(m, ArucoMarker) and m.id == 47]
    if len(servos) < 2:
        raise RuntimeError("Need two servo markers with id 47")
    p1 = np.array(servos[0].center, dtype=float)
    p2 = np.array(servos[1].center, dtype=float)
    mid = (p1 + p2) / 2.0
    return int(mid[0]), int(mid[1])


def build_grid(
    mgr: ArenaManager,
    num: int,
    *,
    margin_mm: float | None = None,
) -> List[Tuple[float, float]]:
    """Return ``num`` grid points inside the manager working area.

    The polygon describing the working area is obtained from
    :meth:`ArenaManager.get_working_area_polygon`.  Points are kept
    at least ``margin_mm`` away from the polygon boundary.
    """

    poly_px = mgr.get_working_area_polygon()
    if len(poly_px) < 3:
        raise RuntimeError("working area polygon not available")

    if margin_mm is None:
        margin_mm = getattr(mgr, "cal_margin_mm", 5.0)

    poly_mm = [mgr.pixel_to_mm(pt) for pt in poly_px]
    xs = [p[0] for p in poly_mm]
    ys = [p[1] for p in poly_mm]

    min_x = min(xs) + margin_mm
    max_x = max(xs) - margin_mm
    min_y = min(ys) + margin_mm
    max_y = max(ys) - margin_mm

    grid_x = int(math.ceil(math.sqrt(num)))
    grid_y = int(math.ceil(num / grid_x))
    grid_x = max(1, grid_x)
    grid_y = max(1, grid_y)

    cand_pts: List[Tuple[float, float]] = []
    for j in range(grid_y):
        y = (
            min_y
            if grid_y == 1
            else min_y + (max_y - min_y) * j / (grid_y - 1)
        )
        for i in range(grid_x):
            x = (
                min_x
                if grid_x == 1
                else min_x + (max_x - min_x) * i / (grid_x - 1)
            )
            cand_pts.append((float(x), float(y)))

    poly_np = np.array(poly_px, dtype=np.int32)
    px_per_mm = float(np.linalg.norm(mgr.calibration["u_x"]))
    margin_px = margin_mm * px_per_mm

    pts: List[Tuple[float, float]] = []
    for pt in cand_pts:
        px_pt = mgr.mm_to_pixel(pt)
        dist = cv2.pointPolygonTest(poly_np, px_pt, True)
        if dist > margin_px:
            pts.append(pt)
            if len(pts) >= num:
                break

    return pts


def main() -> None:
    api = GameAPI()
    api.set_cam_source(0, width=1280, height=720, fourcc="MJPG")
    api.start()
    api.connect_pico()

    print("Waiting for detections…")
    mgr: ArenaManager | None = None
    arena: Arena | None = None
    servo_origin: Tuple[int, int] | None = None
    start = time.time()
    while time.time() - start < 10:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
            mgr = next((c for c in api.plotclocks.values() if isinstance(c, ArenaManager)), None)
        if mgr:
            if arena is None:
                walls = [d for d in detections if isinstance(d, ArucoWall)]
                if walls:
                    arena = Arena(walls)
                    mgr.set_arena(arena)
            if servo_origin is None and len([d for d in detections if isinstance(d, ArucoMarker) and d.id == 47]) >= 2:
                servo_origin = compute_servo_origin(detections)
        if mgr and servo_origin:
            break
    if mgr is None or servo_origin is None:
        print("Required markers not found")
        return

    def _get_dets():
        with api.lock:
            return api.balls + api.arucos

    print("Calibrating Arena Manager…")
    calibrate_clocks([mgr], _get_dets)
    print("Calibration complete")

    grid_mm = build_grid(mgr, NUM_POINTS)
    grid_px = [mgr.mm_to_pixel(pt) for pt in grid_mm]

    results: List[Tuple[float, float, int, int]] = []
    running = False
    idx = 0

    root = tk.Tk()
    root.title("Arena Manager Calibration Map")
    label = tk.Label(root)
    label.pack()
    btn = tk.Button(root, text="Start Calibration")
    btn.pack()

    def on_start():
        nonlocal running
        running = True
        btn.config(state=tk.DISABLED)
    btn.configure(command=on_start)

    def update_loop() -> None:
        nonlocal idx, running
        frame = api.get_annotated_frame()
        if frame is not None:
            if arena:
                draw_arena(frame, arena)
            mgr.draw_working_area(frame)
            cv2.circle(frame, servo_origin, 5, (255, 0, 0), -1)
            with api.lock:
                markers = list(api.arucos)
            servos = [m for m in markers if isinstance(m, ArucoMarker) and m.id == 47]
            for s in servos:
                cv2.circle(frame, s.center, 5, (0, 255, 255), 2)
            for j, pt in enumerate(grid_px):
                col = (0, 255, 0) if j <= idx else (0, 0, 255)
                cv2.circle(frame, pt, 3, col, -1)
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im_tk = ImageTk.PhotoImage(Image.fromarray(img))
            label.configure(image=im_tk)
            label.image = im_tk

        if running and idx < len(grid_mm):
            target = grid_mm[idx]
            mgr.send_command(f"p.setXY({target[0]}, {target[1]})")
            time.sleep(1.5)
            with api.lock:
                dets = list(api.arucos)
            mgr_marker = next((d for d in dets if isinstance(d, ArucoManager)), None)
            if mgr_marker:
                px = mgr_marker.center[0] - servo_origin[0]
                py = mgr_marker.center[1] - servo_origin[1]
                results.append((target[0], target[1], px, py))
            idx += 1
            if idx >= len(grid_mm):
                with open("calibration_map.csv", "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["cmd_x_mm", "cmd_y_mm", "px_x", "px_y"])
                    w.writerows(results)
                print("Calibration data saved to calibration_map.csv")
                running = False
        if running or idx < len(grid_mm):
            root.after(30, update_loop)
        else:
            root.quit()

    root.after(0, update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
