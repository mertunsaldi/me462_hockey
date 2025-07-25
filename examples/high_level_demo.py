"""Shootout game demo using the high level API.

Rules
-----
1. Discover two PlotClocks from the ArUco detections.
2. Calibrate both clocks.
3. Clock0 acts as attacker and shoots the ball towards a
   fixed target at (100, 0) mm.
4. Clock1 defends using a BallReflector.
5. The round ends when the attacker scenario finishes.
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import cv2
import tkinter as tk
from PIL import Image, ImageTk
from ball_example.game_api import GameAPI
from ball_example.models import ArucoWall, Arena
from ball_example.gadgets import ArenaManager
from high_level import calibrate_clocks, draw_arena

def main() -> None:
    api = GameAPI()
    api.set_cam_source(0)
    api.start()
    api.connect_pico()

    print("Waiting for detections...")
    clocks = []
    arena: Arena | None = None
    required_ids = {0, 1}
    start = time.time()
    while time.time() - start < 5:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
            clocks = [c for c in api.plotclocks.values() if c.device_id in required_ids]
        if arena is None:
            walls = [d for d in detections if isinstance(d, ArucoWall)]
            if walls:
                arena = Arena(walls)
        ids = {c.device_id for c in clocks}
        if required_ids.issubset(ids):
            mgr = next(
                (c for c in clocks if isinstance(c, ArenaManager) and c.device_id == 0),
                None,
            )
            clk1 = next(
                (c for c in clocks if not isinstance(c, ArenaManager) and c.device_id == 1),
                None,
            )
            if mgr and clk1:
                break

    ids = {c.device_id for c in clocks}
    mgr = next((c for c in clocks if isinstance(c, ArenaManager) and c.device_id == 0), None)
    clk1 = next((c for c in clocks if not isinstance(c, ArenaManager) and c.device_id == 1), None)
    if not (mgr and clk1):
        print("Missing required markers: ArenaManager id=0 and PlotClock id=1 must be visible")
        return

    attacker = None
    defender = None

    if clocks:
        # Ensure no new PlotClock instances are being created before
        # starting calibration.  The detection thread may add clocks a
        # little later which can clash with calibration commands.
        stable_start = time.time()
        last_count = len(clocks)
        while time.time() - stable_start < 1.0:
            time.sleep(0.1)
            with api.lock:
                clocks = [c for c in api.plotclocks.values() if c.device_id in required_ids]
            if len(clocks) == last_count:
                break
            last_count = len(clocks)
            stable_start = time.time()

        def _get_dets():
            with api.lock:
                return api.balls + api.arucos

        print("Calibrating clocks…")
        calibrate_clocks(clocks, _get_dets)

        attacker = clk1.attack(api.frame_size, (100.0, 0.0))
        defender = None

        attacker.on_start()
        if defender:
            defender.on_start()

    root = tk.Tk()
    root.title("High Level Demo")
    label = tk.Label(root)
    label.pack()

    def update_loop() -> None:
        with api.lock:
            dets = api.balls + api.arucos
        if attacker:
            attacker.update(dets)
        if defender:
            defender.update(dets)

        frame = api.get_annotated_frame()
        if frame is not None:
            if arena:
                draw_arena(frame, arena)
            for c in clocks:
                c.draw_working_area(frame)
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            im_tk = ImageTk.PhotoImage(im_pil)
            label.configure(image=im_tk)
            label.image = im_tk

        if attacker and attacker.finished:
            root.quit()
        else:
            root.after(30, update_loop)

    root.after(0, update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()

