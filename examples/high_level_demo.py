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

import time
import cv2
import tkinter as tk
from PIL import Image, ImageTk
from ball_example.game_api import GameAPI
from high_level import discover_plotclocks, calibrate_clocks

def main() -> None:
    api = GameAPI()
    api.start()

    print("Waiting for detections...")
    clocks = []
    start = time.time()
    while time.time() - start < 5.0 and len(clocks) < 1:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
        clocks = discover_plotclocks(detections)

    if not clocks:
        raise RuntimeError("No PlotClocks detected")

    def _get_dets():
        with api.lock:
            return api.balls + api.arucos

    print("Calibrating clocks…")
    calibrate_clocks(clocks, _get_dets)

    attacker = clocks[0].attack(api.frame_size, (100.0, 0.0))
    defender = clocks[1].defend(api.frame_size) if len(clocks) > 1 else None

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
        attacker.update(dets)
        if defender:
            defender.update(dets)

        frame = api.get_annotated_frame()
        if frame is not None:
            for c in clocks:
                c.draw_working_area(frame)
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            im_tk = ImageTk.PhotoImage(im_pil)
            label.configure(image=im_tk)
            label.image = im_tk

        if not attacker.finished:
            root.after(30, update_loop)
        else:
            root.quit()

    root.after(0, update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()

