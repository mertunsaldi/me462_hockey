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
import re
import cv2
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk
from ball_example.game_api import GameAPI
from ball_example.models import ArucoWall, Arena
from high_level import calibrate_clocks, draw_arena


class AutocompleteEntry(ttk.Entry):
    """Simple Entry widget with Listbox suggestions below."""

    def __init__(self, master, suggest_func, **kwargs):
        super().__init__(master, **kwargs)
        self.suggest_func = suggest_func
        self.var = tk.StringVar()
        self.configure(textvariable=self.var)
        self.var.trace_add("write", self._update_list)
        self.listbox: tk.Listbox | None = None
        self.bind("<FocusOut>", lambda e: self._hide_list())

    def _update_list(self, *_):
        value = self.var.get()
        suggestions = self.suggest_func(value)
        if suggestions:
            if self.listbox is None:
                self.listbox = tk.Listbox(self.master, height=min(6, len(suggestions)))
                self.listbox.bind("<<ListboxSelect>>", self._select_item)
                self.listbox.place(in_=self, relx=0, rely=1, relwidth=1)
            self.listbox.delete(0, tk.END)
            for s in suggestions:
                self.listbox.insert(tk.END, s)
        else:
            self._hide_list()

    def _hide_list(self) -> None:
        if self.listbox is not None:
            self.listbox.destroy()
            self.listbox = None

    def _select_item(self, _):
        if self.listbox is not None:
            idx = self.listbox.curselection()
            if idx:
                self.var.set(self.listbox.get(idx[0]))
            self._hide_list()
            self.icursor(tk.END)

def main() -> None:
    api = GameAPI()
    api.set_cam_source(0)
    api.start()

    print("Waiting for detections...")
    clocks = []
    arena: Arena | None = None
    start = time.time()
    while time.time() - start < 3 and len(clocks) < 2:
        time.sleep(0.1)
        with api.lock:
            detections = list(api.arucos)
            clocks = list(api.plotclocks.values())
        if arena is None:
            walls = [d for d in detections if isinstance(d, ArucoWall)]
            if walls:
                arena = Arena(walls)

    if len(clocks) < 2:
        print(f"Warning: expected 2 PlotClocks, found {len(clocks)}")
        if not clocks:
            print("No PlotClocks detected, continuing without scenarios")

    attacker = None
    defender = None

    if clocks:
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

    # Layout -----------------------------------------------------------------
    video_frame = tk.Frame(root)
    video_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    label = tk.Label(video_frame)
    label.pack(fill=tk.BOTH, expand=True)

    sidebar = tk.Frame(root, width=200)
    sidebar.pack(side=tk.RIGHT, fill=tk.Y)

    connect_btn = tk.Button(sidebar, text="Connect Pico")
    connect_btn.pack(fill=tk.X, pady=4)

    servo_cmds = [
        "getAngle()",
        "setAngle(rad)",
        "calibrateRotation(rad)",
    ]

    p_cmds = [
        "setup(L1,L2,L3,L4,hitDia,servoMarginAngle=2.5/180*math.pi)",
        "getXY()",
        "getMaxX()",
        "getMinY()",
        "getStartAngle()",
        "startPoseCalibration()",
        "goHome()",
        "checkTarget()",
        "gotoXY()",
        "setXY()",
        "setXYrel()",
        "getL1()",
        "getL2()",
        "getL3()",
        "getL4()",
    ]

    clock_ids = [c.device_id for c in clocks if c.device_id is not None]

    def suggest(val: str) -> list[str]:
        if not val or "." not in val:
            return [f"P{i}." for i in clock_ids]
        if re.fullmatch(r"P\d+\.", val):
            return [val + s for s in ("p.", "sr.", "sl.")]
        m = re.match(r"^(P\d+\.(?:p|sr|sl))\.?", val)
        if m:
            prefix = m.group(1)
            cmds = p_cmds if prefix.endswith("p") else servo_cmds
            return [f"{prefix}.{c}" for c in cmds]
        return []

    cmd_entry = AutocompleteEntry(sidebar, suggest)
    cmd_entry.pack(fill=tk.X, pady=4)

    log_text = tk.Text(sidebar, height=10, width=30, state=tk.DISABLED)
    log_text.pack(fill=tk.BOTH, expand=True)

    pico_connected = False

    def connect_pico() -> None:
        nonlocal pico_connected
        try:
            api.connect_pico()
            connect_btn.config(text="Pico Connected", state=tk.DISABLED)
            pico_connected = True
        except Exception as e:
            messagebox.showerror("Connect Error", str(e))

    connect_btn.config(command=connect_pico)

    def send_cmd(event=None) -> None:
        if not pico_connected:
            return
        cmd = cmd_entry.get().strip()
        if not cmd:
            return
        try:
            api.send_cmd(cmd)
            cmd_entry.delete(0, tk.END)
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    cmd_entry.bind("<Return>", send_cmd)

    def update_log() -> None:
        lines = api.read_pico_lines()
        if lines:
            log_text.config(state=tk.NORMAL)
            for line in lines:
                log_text.insert(tk.END, line + "\n")
            log_text.config(state=tk.DISABLED)
            log_text.see(tk.END)
        root.after(500, update_log)

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
    root.after(500, update_log)
    root.mainloop()


if __name__ == "__main__":
    main()

