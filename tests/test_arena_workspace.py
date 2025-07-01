import os, sys, pytest
pytest.importorskip("cv2")
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ball_example.gadgets import ArenaManager
from ball_example.models import ArucoWall, Arena


def _wall(x, y):
    # create a simple square marker 24x24 pixels at (x,y)
    corners = [(x, y), (x + 24, y), (x + 24, y + 24), (x, y + 24)]
    return ArucoWall(0, corners, (x + 12, y + 12))


def test_draw_working_area_uses_arena_polygon():
    arena = Arena([
        _wall(10, 10),
        _wall(110, 10),
        _wall(110, 110),
        _wall(10, 110),
    ])
    mgr = ArenaManager()
    mgr.set_arena(arena)
    frame = np.zeros((200, 200, 3), dtype=np.uint8)
    mgr.draw_working_area(frame)
    assert frame.sum() > 0


def test_plotclock_draw_working_area_shows_axes():
    mgr = ArenaManager()
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (50, 50),
    }
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    mgr.draw_working_area(frame)
    # expect arrows roughly along positive x and y directions
    assert frame[50, 60:80].sum() > 0
    assert frame[50:80, 50].sum() > 0


def test_arena_manager_blocks_out_of_bounds_move():
    arena = Arena([
        _wall(0, 0),
        _wall(100, 0),
        _wall(100, 100),
        _wall(0, 100),
    ])
    mgr = ArenaManager(device_id=1)
    mgr.set_arena(arena)

    # identity calibration so mm == px
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

    mgr.setXY_updated_manager(50, 50)
    assert master.sent == ["P1.p.setXY(50,50)"]

    master.sent.clear()
    mgr.setXY_updated_manager(150, 150)
    assert master.sent == []
