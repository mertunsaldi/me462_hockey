"""Low level API usage example.

This script shows how to access raw detections and send direct commands to a
PlotClock.  A simple Tkinter GUI displays the annotated camera feed using only
low level methods from :class:`GameAPI`.
"""

import time
import cv2
from ball_example.game_api import GameAPI
import tkinter as tk
from PIL import Image, ImageTk


def main() -> None:
    """Run the low level demo."""
    api = GameAPI()
    api.set_cam_source(1)
    api.start()

    root = tk.Tk()
    root.title("Low Level Demo")
    label = tk.Label(root)
    label.pack()

    def update_frame() -> None:
        frame = api.get_annotated_frame()
        if frame is not None:
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            im_tk = ImageTk.PhotoImage(im_pil)
            label.configure(image=im_tk)
            label.image = im_tk
        root.after(30, update_frame)

    root.after(0, update_frame)

    while True:
        time.sleep(0.1)
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
        if not root.winfo_exists():
            break

    root.mainloop()


if __name__ == "__main__":
    main()

