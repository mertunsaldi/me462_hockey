import os, sys, threading, types
import pytest

pytest.importorskip("numpy")
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ball_example.gadgets import ArenaManager

class DummyTimer:
    def __init__(self, delay, func, *a, **kw):
        self.delay = delay
        self.func = func
        self.args = a
        self.kw = kw
    def start(self):
        self.func(*self.args, **self.kw)


def test_setXY_updated_manager(monkeypatch):
    monkeypatch.setattr(threading, "Timer", DummyTimer)

    mgr = ArenaManager(device_id=1)
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    mgr._manager_center_px = (0, 0)

    class DummyMaster:
        def __init__(self):
            self.sent = []
        def send_command(self, cmd):
            self.sent.append(cmd)

    master = DummyMaster()
    mgr.master = master

    mgr.setXY_updated_manager(100, 50)
    assert master.sent == ["P1.p.setXY(100, 50)", "P1.p.setXYrel(100.0, 50.0)"]
    assert mgr._pending_target_mm is None
