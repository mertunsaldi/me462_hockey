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

cv2_stub = types.SimpleNamespace()
cv2_stub.createBackgroundSubtractorMOG2 = lambda *a, **k: object()
cv2_stub.COLOR_BGR2GRAY = 0
cv2_stub.INTER_NEAREST = 0
cv2_stub.VideoCapture = lambda *a, **k: types.SimpleNamespace(
    read=lambda: (False, None),
    isOpened=lambda: True,
    release=lambda: None,
    set=lambda *args, **k: None,
    get=lambda *args, **k: 0,
)
cv2_stub.CAP_PROP_FOURCC = 6
cv2_stub.CAP_PROP_FRAME_WIDTH = 3
cv2_stub.CAP_PROP_FRAME_HEIGHT = 4
cv2_stub.CAP_PROP_FPS = 5
cv2_stub.VideoWriter_fourcc = lambda *a: 0
cv2_stub.circle = lambda *a, **k: None
cv2_stub.putText = lambda *a, **k: None
cv2_stub.FONT_HERSHEY_SIMPLEX = 0
cv2_stub.line = lambda *a, **k: None
cv2_stub.arrowedLine = lambda *a, **k: None
cv2_stub.polylines = lambda *a, **k: None
cv2_stub.aruco = types.SimpleNamespace(
    DICT_6X6_100=0,
    DICT_4X4_100=1,
    getPredefinedDictionary=lambda x: x,
    DetectorParameters=lambda: object(),
    detectMarkers=lambda *a, **k: ([], None, None),
)
cv2_stub.cvtColor = lambda f, code: f
cv2_stub.resize = lambda f, dsize=None, fx=None, fy=None, interpolation=None: f
sys.modules.setdefault("cv2", cv2_stub)

from ball_example.gadgets import ArenaManager
from ball_example.models import ArucoManager, ArucoHitter
from ball_example.game_api import GameAPI
import numpy as np

def _marker(center=(0,0)):
    return ArucoManager(1, [(0,0)]*4, center)

def test_arena_manager_calibration_progresses():
    clock = ArenaManager(device_id=1)
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


def test_plotclock_accepts_manual_cal_points():
    pts = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    clock = ArenaManager(cal_points_mm=pts)
    assert clock._mm_pts == pts


def test_gameapi_uses_cal_points_per_id(monkeypatch):
    pts0 = [(1.0, 1.0), (2.0, 1.0), (1.5, 2.0)]
    pts1 = [(-1.0, 0.0), (-2.0, 0.0), (-1.5, 1.0)]
    api = GameAPI(cal_points_by_id={0: pts0, 1: pts1})
    api.master_pico = None
    api.pico_connected = True

    mgr = ArucoManager(0, [(0, 0)] * 4, (0, 0))
    clk = ArucoHitter(1, [(0, 0)] * 4, (0, 0))

    monkeypatch.setattr(
        "ball_example.detectors.ArucoDetector.detect",
        lambda f: [mgr, clk],
    )
    monkeypatch.setattr(
        "ball_example.trackers.BallTracker.update",
        lambda self, frame, mask=None: [],
    )
    monkeypatch.setattr(api.mask_pipe, "get_masked_frame", lambda: None)

    frame = np.zeros((10, 10, 3), dtype=np.uint8)
    api._process_annotated(frame)

    assert isinstance(api.plotclocks[0], ArenaManager)
    assert api.plotclocks[0]._mm_pts == pts0
    assert api.plotclocks[1]._mm_pts == pts1


