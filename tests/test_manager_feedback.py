import os, sys, types, pytest, time

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

    mgr.record_manager_position((0, 0))

    def fake_sleep(t):
        mgr.record_manager_position((8, 18))

    original_sleep = time.sleep
    time.sleep = fake_sleep
    try:
        mgr.setXY_updated_manager(10, 20)
    finally:
        time.sleep = original_sleep

    assert master.sent == ["P1.p.setXY(10, 20)", "P1.p.setXYrel(2.0, 2.0)"]
