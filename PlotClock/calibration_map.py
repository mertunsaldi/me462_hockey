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


def eval_poly(coeffs: np.ndarray, x: float, y: float) -> float:
    """Evaluate a 2D quadratic polynomial with coefficients ``coeffs``."""
    return (
        coeffs[0]
        + coeffs[1] * x
        + coeffs[2] * y
        + coeffs[3] * x * x
        + coeffs[4] * x * y
        + coeffs[5] * y * y
    )


def fit_polynomial(
    inputs: np.ndarray, outputs: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Fit a 2D quadratic polynomial mapping ``inputs`` -> ``outputs``."""
    x = inputs[:, 0]
    y = inputs[:, 1]
    A = np.column_stack(
        [np.ones(len(inputs)), x, y, x**2, x * y, y**2]
    )
    coeff_x, *_ = np.linalg.lstsq(A, outputs[:, 0], rcond=None)
    coeff_y, *_ = np.linalg.lstsq(A, outputs[:, 1], rcond=None)
    return coeff_x, coeff_y


def compute_servo_transform(
    markers: List[ArucoMarker],
) -> Tuple[Tuple[int, int], np.ndarray, np.ndarray, float]:
    """Return the servo origin, axes and marker distance in pixels.

    The axes are aligned with the arena manager's coordinate frame so that the
    resulting pixel coordinates match the orientation used for ``setXY``
    commands.
    """

    servos = [m for m in markers if isinstance(m, ArucoMarker) and m.id == 47]
    mgr = next((m for m in markers if isinstance(m, ArucoManager)), None)
    if len(servos) < 2 or mgr is None:
        raise RuntimeError("Need two servo markers (id 47) and manager marker")

    servos = sorted(servos, key=lambda m: m.center[0])
    p1 = np.array(servos[0].center, dtype=float)
    p2 = np.array(servos[1].center, dtype=float)
    mid = (p1 + p2) / 2.0
    dx = -np.abs(p2 - p1)
    servo_px_dist = float(np.linalg.norm(p2 - p1))
    norm = np.linalg.norm(dx)
    if norm < 1e-6:
        raise RuntimeError("Servo markers too close")
    e_x = dx / norm
    e_y = np.array([-e_x[1], e_x[0]])
    mgr_vec = np.array(mgr.center, dtype=float) - mid
    if np.dot(mgr_vec, e_y) < 0:
        e_y = -e_y
    if e_x[0] * e_y[1] - e_x[1] * e_y[0] > 0:
        e_x = -e_x

    origin = (int(mid[0]), int(mid[1]))
    return origin, e_x, e_y, servo_px_dist


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

    poly_np = np.array(poly_px, dtype=np.int32)
    px_per_mm = float(np.linalg.norm(mgr.calibration["u_x"]))
    margin_px = margin_mm * px_per_mm

    # Estimate grid spacing from the polygon area to obtain a roughly
    # uniform distribution of points across irregular shapes.
    area = float(cv2.contourArea(np.array(poly_mm, dtype=np.float32)))
    if area <= 0:
        raise RuntimeError("invalid working area polygon")
    spacing = math.sqrt(area / num)

    # Decrease spacing slightly so that we generate more candidates than
    # strictly required.  This helps when parts of the grid fall outside the
    # working polygon.
    spacing *= 0.9

    rows: List[List[Tuple[float, float]]] = []
    y = min_y
    while y <= max_y:
        row: List[Tuple[float, float]] = []
        x = min_x
        while x <= max_x:
            pt = (float(x), float(y))
            px_pt = mgr.mm_to_pixel(pt)
            dist = cv2.pointPolygonTest(poly_np, px_pt, True)
            if dist > margin_px:
                row.append(pt)
            x += spacing
        if row:
            rows.append(row)
        y += spacing

    pts: List[Tuple[float, float]] = []
    for i, row in enumerate(rows):
        ordered = sorted(row, key=lambda p: p[0], reverse=(i % 2 == 1))
        pts.extend(ordered)

    if len(pts) < num:
        raise RuntimeError(
            f"Only {len(pts)} points found inside working area; consider reducing the margin"
        )

    return pts[:num]


def main() -> None:
    api = GameAPI()
    api.set_cam_source(0, width=1280, height=720, fourcc="MJPG")
    api.start()
    api.connect_pico()

    print("Waiting for detections…")
    mgr: ArenaManager | None = None
    arena: Arena | None = None
    servo_origin: Tuple[int, int] | None = None
    servo_ex: np.ndarray | None = None
    servo_ey: np.ndarray | None = None
    servo_px_dist: float | None = None
    start = time.time()
    while time.time() - start < 10:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
            mgr = next(
                (c for c in api.plotclocks.values() if isinstance(c, ArenaManager)),
                None,
            )
        if mgr:
            if arena is None:
                walls = [d for d in detections if isinstance(d, ArucoWall)]
                if walls:
                    arena = Arena(walls)
                    mgr.set_arena(arena)
            if (
                servo_origin is None
                and len(
                    [d for d in detections if isinstance(d, ArucoMarker) and d.id == 47]
                )
                >= 2
                and next((d for d in detections if isinstance(d, ArucoManager)), None)
                is not None
            ):
                servo_origin, servo_ex, servo_ey, servo_px_dist = compute_servo_transform(detections)
        if mgr and servo_origin is not None:
            break
    if (
        mgr is None
        or servo_origin is None
        or servo_ex is None
        or servo_ey is None
        or servo_px_dist is None
    ):
        print("Required markers not found")
        return

    def _get_dets():
        with api.lock:
            return api.balls + api.arucos

    print("Calibrating Arena Manager…")
    calibrate_clocks([mgr], _get_dets)
    print("Calibration complete")

    if not mgr.calibration:
        print("Arena Manager failed to calibrate")
        return

    poly_pts = mgr.get_working_area_polygon()
    if len(poly_pts) < 3:
        print("Working area polygon not available")
        return

    grid_mm = build_grid(mgr, NUM_POINTS)
    grid_px = [mgr.mm_to_pixel(pt) for pt in grid_mm]

    results: List[Tuple[float, float, float, float]] = []
    running = False
    idx = 0
    moving = False
    move_start = 0.0
    current_target: Tuple[float, float] | None = None
    coeff_x: np.ndarray | None = None
    coeff_y: np.ndarray | None = None
    correct_mode = False

    root = tk.Tk()
    root.title("Arena Manager Calibration Map")
    label = tk.Label(root)
    label.pack()
    btn = tk.Button(root, text="Start Calibration")
    btn.pack()
    btn_correct = tk.Button(root, text="Go to Corrected Points", state=tk.DISABLED)
    btn_correct.pack()

    def on_start():
        nonlocal running, correct_mode, idx
        running = True
        correct_mode = False
        idx = 0
        btn.config(state=tk.DISABLED)

    btn.configure(command=on_start)

    def on_correct():
        nonlocal running, correct_mode, idx
        if coeff_x is None or coeff_y is None:
            return
        running = True
        correct_mode = True
        idx = 0
        btn_correct.config(state=tk.DISABLED)

    btn_correct.configure(command=on_correct)

    def update_loop() -> None:
        nonlocal idx, running, moving, move_start, current_target, coeff_x, coeff_y
        frame = api.get_annotated_frame()
        if frame is not None:
            if arena:
                draw_arena(frame, arena)
            mgr.draw_working_area(frame)
            cv2.circle(frame, servo_origin, 5, (255, 0, 0), -1)
            axis_len = 40
            end_x = (
                int(servo_origin[0] + servo_ex[0] * axis_len),
                int(servo_origin[1] + servo_ex[1] * axis_len),
            )
            end_y = (
                int(servo_origin[0] + servo_ey[0] * axis_len),
                int(servo_origin[1] + servo_ey[1] * axis_len),
            )
            cv2.arrowedLine(frame, servo_origin, end_x, (255, 0, 0), 2, tipLength=0.2)
            cv2.arrowedLine(frame, servo_origin, end_y, (0, 0, 255), 2, tipLength=0.2)
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

        if running:
            if not moving and idx < len(grid_mm):
                current_target = grid_mm[idx]
                tx, ty = current_target
                if correct_mode and coeff_x is not None and coeff_y is not None:
                    tx = eval_poly(coeff_x, current_target[0], current_target[1])
                    ty = eval_poly(coeff_y, current_target[0], current_target[1])
                mgr.send_command(f"p.setXY({tx}, {ty})")
                move_start = time.time()
                moving = True
            elif moving and time.time() - move_start >= 1.5:
                with api.lock:
                    dets = list(api.arucos)
                mgr_marker = next(
                    (d for d in dets if isinstance(d, ArucoManager)), None
                )
                if mgr_marker and current_target is not None:
                    delta = np.array(mgr_marker.center, dtype=float) - np.array(
                        servo_origin, dtype=float
                    )
                    px = float(np.dot(delta, servo_ex))
                    py = float(np.dot(delta, servo_ey))
                    if not correct_mode:
                        results.append(
                            (current_target[0], current_target[1], px, py)
                        )
                idx += 1
                moving = False
                if idx >= len(grid_mm):
                    if not correct_mode:
                        with open("calibration_map.csv", "w", newline="") as f:
                            w = csv.writer(f)
                            w.writerow(["cmd_x_mm", "cmd_y_mm", "px_x", "px_y"])
                            w.writerows(results)
                        print("Calibration data saved to calibration_map.csv")
                        mm_per_px = ArenaManager.SERVO_MM_DIST / servo_px_dist
                        inp = np.array([[r[2] * mm_per_px, r[3] * mm_per_px] for r in results])
                        out = np.array([[r[0], r[1]] for r in results])
                        coeff_x, coeff_y = fit_polynomial(inp, out)
                        print("x coeffs:", coeff_x.tolist())
                        print("y coeffs:", coeff_y.tolist())
                        print("Use eval_poly(coeff_x, x, y) and eval_poly(coeff_y, x, y) for corrected setXY values")
                    btn_correct.config(state=tk.NORMAL)
                    running = False

        if running or idx < len(grid_mm) or moving:
            root.after(30, update_loop)
        else:
            root.quit()

    root.after(0, update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
