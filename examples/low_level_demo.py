"""Low level API usage example.

This script shows how to access raw detections and send direct commands to a
PlotClock. A simple Tkinter GUI displays the camera feed that we annotate
ourselves using the low level data exposed by :class:`GameAPI`.
"""

import time
import cv2
import tkinter as tk
from PIL import Image, ImageTk
from ball_example.game_api import GameAPI
from ball_example.renderers import render_overlay


def main() -> None:
    """Run the low level demo."""
    api = GameAPI()
    api.set_cam_source(0)
    api.start()

    root = tk.Tk()
    root.title("Low Level Demo")
    label = tk.Label(root)
    label.pack()

    def update_frame() -> None:
        frame = api.raw_pipe.get_raw_frame()
        if frame is not None:
            with api.lock:
                balls = list(api.balls)
                markers = list(api.arucos)
            annotated = render_overlay(frame, balls, markers)
            img = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            im_tk = ImageTk.PhotoImage(im_pil)
            label.configure(image=im_tk)
            label.image = im_tk
        root.after(30, update_frame)

    def poll_state() -> None:
        with api.lock:
            balls = list(api.balls)
            markers = list(api.arucos)
        print("balls", [b.center for b in balls])
        print("markers", [m.id for m in markers])

        if balls and not api.pico_connected:
            try:
                api.connect_pico()
                api.send_cmd("home")
            except Exception as e:
                print("pico error", e)

        root.after(100, poll_state)

    root.after(0, update_frame)
    root.after(0, poll_state)
    root.mainloop()


if __name__ == "__main__":
    main()

