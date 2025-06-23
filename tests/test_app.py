import os, sys, json
import pytest

pytest.importorskip("cv2")
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from ball_example import app as hockey_app


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


def test_stats_extended_fields():
    client = hockey_app.app.test_client()
    r = client.get("/stats")
    assert r.status_code == 200
    data = r.get_json()
    assert "ball_details" in data


def test_manual_mode_endpoints():
    client = hockey_app.app.test_client()
    r = client.get("/manual_params")
    assert r.status_code == 200
    data = r.get_json()
    assert "manual" in data
    r2 = client.post("/manual_mode", json={"enable": True})
    assert r2.status_code == 200
    r3 = client.post("/manual_params", json={"param": "edge_density", "value": 0.2})
    assert r3.status_code == 200
    r4 = client.post("/manual_mode", json={"enable": False})
    assert r4.status_code == 200


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


def test_game_api_set_cam_source():
    from ball_example.game_api import GameAPI
    api = GameAPI()
    api.set_cam_source(0)
    assert api.camera.src == 0

