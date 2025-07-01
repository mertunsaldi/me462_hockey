import os, sys, types, pytest

pytest.importorskip("numpy")

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

serial_stub = types.SimpleNamespace(
    SerialException=Exception,
    tools=types.SimpleNamespace(list_ports=types.SimpleNamespace(comports=lambda: [])),
)
sys.modules.setdefault("serial", serial_stub)
sys.modules.setdefault("serial.tools", serial_stub.tools)
sys.modules.setdefault("serial.tools.list_ports", serial_stub.tools.list_ports)

import numpy as np

from ball_example.gadgets import ArenaManager
from ball_example.models import ArucoMarker, ArucoManager


def _servo(center=(0, 0)):
    return ArucoMarker(47, [(0, 0)] * 4, center)


def _manager(center=(0, 0)):
    return ArucoManager(0, [(0, 0)] * 4, center)


def test_arena_manager_calibration_new_logic():
    mgr = ArenaManager()
    dets = [
        _servo((10, 20)),
        _servo((110, 20)),
        _manager((60, 120)),
    ]

    cal = mgr.calibrate(dets)
    assert cal is not None
    assert pytest.approx(cal["servo_px_dist"], rel=1e-6) == 100.0
    np.testing.assert_allclose(cal["u_x"], np.array([100 / 148.0, 0.0]), atol=1e-6)
    np.testing.assert_allclose(cal["u_y"], np.array([0.0, 100 / 148.0]), atol=1e-6)
    assert cal["origin_px"] == (60, 20)


