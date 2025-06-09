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


def test_load_scenario_and_message(tmp_path):
    scenario_code = (
        "from ball_example.scenarios import Scenario\n"
        "class ClientScenario(Scenario):\n"
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
