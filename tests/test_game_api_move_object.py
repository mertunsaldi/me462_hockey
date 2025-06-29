import os, sys
import pytest

pytest.importorskip("numpy")
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ball_example.game_api import GameAPI
from ball_example.gadgets import ArenaManager
from ball_example.models import Ball


def test_game_api_move_object_finish_clears_mode():
    api = GameAPI()
    mgr = ArenaManager(device_id=1)
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    api.plotclocks[1] = mgr
    ball = Ball((0, 0), 5)
    api.balls.append(ball)
    api.start_move_object(mgr, ball, (10, 20))
    sc = api.clock_scenarios[1]
    sc.on_start()

    # complete scenario steps
    for _ in range(4):
        sc._last_time -= sc.WAIT_TIME + 0.1
        api.update(np.zeros((10, 10, 3), dtype=np.uint8), [], [])

    assert 1 not in api.clock_scenarios
    assert 1 not in api.clock_modes

