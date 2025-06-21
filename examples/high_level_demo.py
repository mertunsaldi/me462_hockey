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
from ball_example.game_api import GameAPI
from ball_example.high_level import (
    discover_plotclocks,
    calibrate_clocks,
)

api = GameAPI()
api.start()

# gather initial detections
print("Waiting for detections...")
time.sleep(2.0)
with api.lock:
    detections = list(api.arucos)

clocks = discover_plotclocks(detections)

def _get_dets():
    with api.lock:
        return api.balls + api.arucos

print("Calibrating clocks…")
calibrate_clocks(clocks, _get_dets)

attacker = clocks[0].attack(api.frame_size, (100.0, 0.0))
if len(clocks) > 1:
    defender = clocks[1].defend(api.frame_size)
else:
    defender = None

attacker.on_start()
if defender:
    defender.on_start()

while not attacker.finished:
    with api.lock:
        dets = api.balls + api.arucos
    attacker.update(dets)
    if defender:
        defender.update(dets)

    frame = api.get_annotated_frame()
    if frame is not None:
        for c in clocks:
            c.draw_working_area(frame)
        cv2.imshow("demo", frame)
        if cv2.waitKey(1) == 27:
            break

cv2.destroyAllWindows()

