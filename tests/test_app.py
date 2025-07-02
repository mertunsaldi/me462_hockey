import os, sys, json
import pytest

pytest.importorskip("cv2")
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from ball_example import app as hockey_app
from ball_example.gadgets import ArenaManager
import numpy as np


def test_load_commands_invalid():
    client = hockey_app.app.test_client()
    resp = client.post("/load_commands", json={"foo": 1})
    assert resp.status_code == 400


def test_load_commands_ok():
    client = hockey_app.app.test_client()
    resp = client.post(
        "/load_commands",
        json={"commands": [{"action": "calibrate", "clock": 0}]},
    )
    assert resp.status_code == 200
    assert resp.get_json()["status"] == "ok"


def test_stop_scenario_endpoint():
    client = hockey_app.app.test_client()
    r = client.post("/stop_scenario")
    assert r.status_code == 200


def test_load_scenario_and_message(tmp_path):
    scenario_code = (
        "from ball_example.scenarios import Scenario\n"
        "class ClientScenario(Scenario):\n"
        "    def __init__(self, plotclock, frame_size):\n"
        "        pass\n"
        "    def update(self, d): pass\n"
        "    def process_message(self, m):\n"
        "        self.last = m\n"
    )
    path = tmp_path / "cs.py"
    path.write_text(scenario_code)
    client = hockey_app.app.test_client()
    resp = client.post("/load_scenario", json={"path": str(path)})
    assert resp.status_code == 200

    msg_resp = client.post("/send_message", json={"foo": 1})
    assert msg_resp.status_code == 200


def test_debug_data_endpoint_returns_debug_info():
    client = hockey_app.app.test_client()
    r = client.get("/debug_data")
    assert r.status_code == 200
    data = r.get_json()
    assert "ball_details" in data
    assert "image_params" in data
    assert "min_radius" in data["image_params"]
    assert "max_radius" in data["image_params"]


def test_manual_mode_endpoints():
    client = hockey_app.app.test_client()
    r = client.get("/manual_params")
    assert r.status_code == 200
    data = r.get_json()
    assert "manual" in data
    assert "min_r" in data and "max_r" in data
    r2 = client.post("/manual_mode", json={"enable": True})
    assert r2.status_code == 200
    r3 = client.post("/manual_params", json={"param": "edge_density", "value": 0.2})
    assert r3.status_code == 200
    r4 = client.post("/manual_params", json={"param": "min_r", "value": 10})
    assert r4.status_code == 200
    r5 = client.post("/manual_mode", json={"enable": False})
    assert r5.status_code == 200


def test_processed_feed_route():
    client = hockey_app.app.test_client()
    r = client.get("/processed_feed")
    assert r.status_code == 200


def test_debug_endpoint():
    client = hockey_app.app.test_client()
    r = client.get("/debug_data")
    assert r.status_code == 200
    data = r.get_json()
    assert "image_params" in data
    assert "gadgets" in data


def test_pico_lines_endpoint():
    client = hockey_app.app.test_client()
    r = client.get("/pico_lines")
    assert r.status_code == 200
    data = r.get_json()
    assert "lines" in data


def test_game_api_set_cam_source():
    from ball_example.game_api import GameAPI
    api = GameAPI()
    api.set_cam_source(0)
    assert api.camera.src == 0


class DummyMaster:
    def __init__(self):
        self.sent = []

    def send_command(self, cmd: str) -> None:
        self.sent.append(cmd)


def test_move_manager_endpoint():
    client = hockey_app.app.test_client()
    api = hockey_app.api
    mgr = ArenaManager(device_id=0, master=DummyMaster())
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    with api.lock:
        api.plotclocks[0] = mgr
        api._current_scenario = None

    try:
        resp = client.post(
            "/move_manager",
            json={"device_id": 0, "x": 10, "y": 20},
        )
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["x_mm"] == 10
        assert data["y_mm"] == 20
        cmd = mgr.master.sent[-1]
        assert cmd.startswith("P0.p.setXY(")
        nums = cmd[len("P0.p.setXY("):-1].split(",")
        assert float(nums[0]) == 10
        assert float(nums[1]) == 20
    finally:
        with api.lock:
            api.plotclocks.pop(0, None)


def test_move_manager_reject_when_scenario_running():
    client = hockey_app.app.test_client()
    api = hockey_app.api
    mgr = ArenaManager(device_id=0, master=DummyMaster())
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    with api.lock:
        api.plotclocks[0] = mgr
        api._current_scenario = object()
        api.scenario_enabled = True

    try:
        resp = client.post(
            "/move_manager",
            json={"device_id": 0, "x": 5, "y": 5},
        )
        assert resp.status_code == 400
        assert api._current_scenario is not None
        assert api.scenario_enabled
    finally:
        with api.lock:
            api.plotclocks.pop(0, None)
            api._current_scenario = None
            api.scenario_enabled = False


def test_move_manager_reject_when_scenario_ready():
    client = hockey_app.app.test_client()
    api = hockey_app.api
    mgr = ArenaManager(device_id=0, master=DummyMaster())
    mgr.calibration = {
        "u_x": np.array([1.0, 0.0]),
        "u_y": np.array([0.0, 1.0]),
        "origin_px": (0, 0),
    }
    with api.lock:
        api.plotclocks[0] = mgr
        api._current_scenario = object()
        api.scenario_enabled = False

    try:
        resp = client.post(
            "/move_manager",
            json={"device_id": 0, "x": 7, "y": 9},
        )
        assert resp.status_code == 400
        assert api._current_scenario is not None
        assert not api.scenario_enabled
    finally:
        with api.lock:
            api.plotclocks.pop(0, None)
            api._current_scenario = None
            api.scenario_enabled = False

