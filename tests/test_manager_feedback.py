import os, sys, types, pytest

pytest.importorskip("numpy")
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

serial_stub = types.SimpleNamespace(
    SerialException=Exception,
    tools=types.SimpleNamespace(list_ports=types.SimpleNamespace(comports=lambda: [])),
)
sys.modules.setdefault("serial", serial_stub)
sys.modules.setdefault("serial.tools", serial_stub.tools)
sys.modules.setdefault("serial.tools.list_ports", serial_stub.tools.list_ports)

from ball_example.gadgets import ArenaManager


class DummyMaster:
    def __init__(self):
        self.sent = []

    def send_command(self, cmd: str) -> None:
        self.sent.append(cmd)


def test_setxy_feedback_sends_relative_move():
    mgr = ArenaManager(device_id=1)
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    master = DummyMaster()
    mgr.master = master

    mgr.update_manager_position((0, 0))
    mgr.send_command("p.setXY(10, 20)")
    assert master.sent == ["P1.p.setXY(10, 20)"]

    master.sent.clear()
    mgr.update_manager_position((8, 18))
    assert mgr.relative_error_px == (2.0, 2.0)
    assert mgr.relative_error_mm == (2.0, 2.0)
    assert master.sent == ["P1.p.setXYrel(2.0, 2.0)"]
