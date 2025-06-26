import os, sys, time, types, pytest

pytest.importorskip("numpy")

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

serial_stub = types.SimpleNamespace(
    SerialException=Exception,
    tools=types.SimpleNamespace(list_ports=types.SimpleNamespace(comports=lambda: [])),
)
sys.modules.setdefault("serial", serial_stub)
sys.modules.setdefault("serial.tools", serial_stub.tools)
sys.modules.setdefault("serial.tools.list_ports", serial_stub.tools.list_ports)

from ball_example.gadgets import ArenaManager
from ball_example.models import ArucoManager

def _marker(center=(0,0)):
    return ArucoManager(1, [(0,0)]*4, center)

def test_arena_manager_calibration_progresses():
    clock = ArenaManager()
    clock._delay = 0
    clock.send_command = lambda *a, **k: None
    # first call sends move to first point
    clock.calibrate([])
    assert clock._cal_state == 1
    # provide marker detections for remaining states
    for i in range(3):
        clock._last_cmd_t = time.time() - 1
        clock.calibrate([_marker((i, i))])
    assert clock.calibration is not None


