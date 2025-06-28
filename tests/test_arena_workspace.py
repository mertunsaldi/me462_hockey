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
