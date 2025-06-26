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
from high_level import discover_plotclocks, calibrate_clocks, draw_arena

def main() -> None:
    api = GameAPI()
    api.set_cam_source(0)
    api.start()

    print("Waiting for detections...")
    clocks = []
    arena: Arena | None = None
    start = time.time()
    while time.time() - start < 0.5 and len(clocks) < 1:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
        clocks = discover_plotclocks(detections)
        if arena is None:
            walls = [d for d in detections if isinstance(d, ArucoWall)]
            if walls:
                arena = Arena(walls)

    if not clocks:
        print("No PlotClocks detected, continuing without scenarios")

    attacker = None
    defender = None

    # if clocks:
    #     def _get_dets():
    #         with api.lock:
    #             return api.balls + api.arucos
    #
    #     print("Calibrating clocks…")
    #     calibrate_clocks(clocks, _get_dets)
    #
    #     attacker = clocks[0].attack(api.frame_size, (100.0, 0.0))
    #     defender = clocks[1].defend(api.frame_size) if len(clocks) > 1 else None
    #
    #     attacker.on_start()
    #     if defender:
    #         defender.on_start()

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

