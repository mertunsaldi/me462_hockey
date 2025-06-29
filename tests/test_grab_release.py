import os, sys, pytest

pytest.importorskip("numpy")
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ball_example.gadgets import ArenaManager
from ball_example.models import Ball, Obstacle


def test_move_object_ball():
    mgr = ArenaManager(device_id=1)
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }

    class DummyMaster:
        def __init__(self):
            self.sent = []

        def send_command(self, cmd: str) -> None:
            self.sent.append(cmd)

    master = DummyMaster()
    mgr.master = master

    ball = Ball((10, 20), 5)
    scenario = mgr.move_object(ball, 100, 200)
    scenario.on_start()

    # step 0 -> move to ball
    scenario.update([])
    assert master.sent == ["P1.p.setXY(10, 20)"]

    # step 1 -> grab (no command)
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert master.sent == ["P1.p.setXY(10, 20)"]

    # step 2 -> move to target
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert master.sent == ["P1.p.setXY(10, 20)", "P1.p.setXY(100, 200)"]

    # step 3 -> release
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert scenario.finished


def test_move_object_obstacle():
    mgr = ArenaManager(device_id=1)
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }

    class DummyMaster:
        def __init__(self):
            self.sent = []

        def send_command(self, cmd: str) -> None:
            self.sent.append(cmd)

    master = DummyMaster()
    mgr.master = master

    obs = Obstacle(0, [], (30, 40))
    scenario = mgr.move_object(obs, 100, 200)
    scenario.on_start()

    # step 0 -> move to object
    scenario.update([])
    assert master.sent == ["P1.p.setXY(30, 40)"]

    # step 1 -> grab (no command)
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert master.sent == ["P1.p.setXY(30, 40)"]

    # step 2 -> move to target
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert master.sent == ["P1.p.setXY(30, 40)", "P1.p.setXY(100, 200)"]

    # step 3 -> release
    scenario._last_time -= scenario.WAIT_TIME + 0.1
    scenario.update([])
    assert scenario.finished
