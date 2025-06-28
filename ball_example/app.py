import cv2, math, threading, time
import numpy as np
from typing import Optional
from flask import Flask, render_template, Response, jsonify, request

if __package__ in (None, ""):
    import os, sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    __package__ = "ball_example"

from .game_api import GameAPI
from .detectors import BallDetector
from .models import ArucoWall, Arena
from .gadgets import ArenaManager
from high_level import calibrate_clocks, draw_arena

app = Flask(__name__, template_folder="templates", static_folder="static")

import logging

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

# Core processing API
api = GameAPI()
api.set_cam_source(2, width=1280, height=720, fourcc="MJPG")

if hasattr(app, "before_first_request"):
    @app.before_first_request
    def _ensure_camera_running():
        api.start()
else:
    @app.before_request
    def _ensure_camera_running():
        api.start()
        
# Manual tuning state
manual_mode = False
detected_clocks = []
detected_arena = None
attacker = None
defender = None
_default_params = {
    "circ": BallDetector.CIRCULARITY_THRESHOLD,
    "area_ratio": BallDetector.AREA_RATIO_THRESHOLD,
    "solidity": BallDetector.SOLIDITY_THRESHOLD,
    "edge_density": BallDetector.EDGE_DENSITY_THRESHOLD,
    "blur": BallDetector.BLUR_KERNEL,
    "sigma": BallDetector.BLUR_SIGMA,
    "min_r": BallDetector.MIN_RADIUS,
    "max_r": BallDetector.MAX_RADIUS,
    "h_low": int(BallDetector.HSV_LOWER[0]),
    "s_low": int(BallDetector.HSV_LOWER[1]),
    "v_low": int(BallDetector.HSV_LOWER[2]),
    "h_up": int(BallDetector.HSV_UPPER[0]),
    "s_up": int(BallDetector.HSV_UPPER[1]),
    "v_up": int(BallDetector.HSV_UPPER[2]),
}
_manual_params = _default_params.copy()

def _apply_params(params):
    BallDetector.CIRCULARITY_THRESHOLD = float(params["circ"])
    BallDetector.AREA_RATIO_THRESHOLD  = float(params["area_ratio"])
    BallDetector.SOLIDITY_THRESHOLD    = float(params["solidity"])
    BallDetector.BLUR_KERNEL = int(params["blur"])
    BallDetector.BLUR_SIGMA  = float(params["sigma"])
    BallDetector.EDGE_DENSITY_THRESHOLD= float(params["edge_density"])
    BallDetector.MIN_RADIUS = int(params["min_r"])
    BallDetector.MAX_RADIUS = int(params["max_r"])
    BallDetector.HSV_LOWER = np.array([params["h_low"], params["s_low"], params["v_low"]], dtype=np.uint8)
    BallDetector.HSV_UPPER = np.array([params["h_up"], params["s_up"], params["v_up"]], dtype=np.uint8)

def _reset_defaults():
    global _manual_params
    _manual_params = _default_params.copy()
    _apply_params(_default_params)


# ------------------------------------------------------------------


@app.route("/start_scenario", methods=["POST"])
def start_scenario():
    """Activate the currently loaded scenario."""
    try:
        api.start_scenario()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/stop_scenario", methods=["POST"])
def stop_scenario():
    """Deactivate the running scenario if any."""
    try:
        api.stop_scenario()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/load_commands", methods=["POST"])
def load_commands():
    data = request.get_json(silent=True) or {}
    commands = data.get("commands")
    if not isinstance(commands, list):
        return jsonify({"status": "error", "message": "commands must be a list"}), 400
    try:
        api.load_commands(commands)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/load_scenario", methods=["POST"])
def load_scenario_route():
    """Load a custom scenario from a Python file path or uploaded file."""
    if "file" in request.files:
        up = request.files["file"]
        path = "/tmp/" + up.filename
        up.save(path)
    else:
        data = request.get_json(silent=True) or {}
        path = data.get("path")
    if not path:
        return jsonify({"status": "error", "message": "no path provided"}), 400
    try:
        api.load_scenario(path)
        return jsonify({"status": "ok"})
    except ScenarioLoadError as e:
        return jsonify({"status": "error", "message": str(e)}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500




def generate_frames():
    while True:
        annotated = api.get_annotated_frame()
        if annotated is None:
            time.sleep(0.01)
            continue

        with api.lock:
            dets = api.balls + api.arucos

        if attacker:
            attacker.update(dets)
        if defender:
            defender.update(dets)

        if detected_arena:
            draw_arena(annotated, detected_arena)
        for c in detected_clocks:
            c.draw_working_area(annotated)

        ok, buf = cv2.imencode(".jpg", annotated)
        if not ok:
            continue

        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
        )


def generate_processed_frames():
    while True:
        mask = api.get_processed_frame()
        if mask is None:
            time.sleep(0.01)
            continue
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        ok, buf = cv2.imencode(".jpg", mask_bgr)
        if not ok:
            continue

        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
        )

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/processed_feed")
def processed_feed():
    return Response(
        generate_processed_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )



@app.route("/debug_data")
def debug_data():
    """Return extended diagnostic information."""
    return jsonify(api.debug_info())


@app.route("/connect_pico", methods=["POST"])
def connect_pico():
    global detected_clocks, detected_arena, attacker, defender
    try:
        api.connect_pico()

        print("Waiting for detections...")
        detected_clocks = []
        detected_arena = None
        required_ids = {0, 1}
        start = time.time()
        while time.time() - start < 5:
            time.sleep(0.1)
            with api.lock:
                detections = list(api.arucos)
                detected_clocks = [
                    c for c in api.plotclocks.values() if c.device_id in required_ids
                ]
            if detected_arena is None:
                walls = [d for d in detections if isinstance(d, ArucoWall)]
                if walls:
                    detected_arena = Arena(walls)
            ids = {c.device_id for c in detected_clocks}
            have_required = required_ids.issubset(ids)
            if have_required:
                mgr = next(
                    (c for c in detected_clocks if isinstance(c, ArenaManager) and c.device_id == 0),
                    None,
                )
                clk1 = next(
                    (c for c in detected_clocks if not isinstance(c, ArenaManager) and c.device_id == 1),
                    None,
                )
                if mgr and clk1:
                    break

        if detected_arena:
            for c in detected_clocks:
                if isinstance(c, ArenaManager):
                    c.set_arena(detected_arena)

        ids = {c.device_id for c in detected_clocks}
        mgr = next((c for c in detected_clocks if isinstance(c, ArenaManager) and c.device_id == 0), None)
        clk1 = next((c for c in detected_clocks if not isinstance(c, ArenaManager) and c.device_id == 1), None)
        if not (mgr and clk1):
            msg = (
                "Missing required markers: ArenaManager id=0 and PlotClock id=1 must be visible"
            )
            print(msg)
            return jsonify({"status": "error", "message": msg}), 400

        if detected_clocks:
            # Wait a moment for any remaining PlotClock objects to finish
            # instantiating.  The detection thread may still be creating
            # gadgets while we exit the loop above which can lead to
            # calibration commands racing with ``__init__`` queries.
            stable_start = time.time()
            last_count = len(detected_clocks)
            while time.time() - stable_start < 1.0:
                time.sleep(0.1)
                with api.lock:
                    detected_clocks = [
                        c
                        for c in api.plotclocks.values()
                        if c.device_id in required_ids
                    ]
                if len(detected_clocks) == last_count:
                    break
                last_count = len(detected_clocks)
                stable_start = time.time()

            def _get_dets():
                with api.lock:
                    return api.balls + api.arucos

            print("Calibrating clocks…")
            calibrate_clocks(detected_clocks, _get_dets)

            attacker = clk1.attack(api.frame_size, (100.0, 0.0))
            defender = None
            attacker.on_start()
            if defender:
                defender.on_start()

        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/send_cmd", methods=["POST"])
def send_cmd():
    try:
        api.connect_pico()
    except Exception:
        return jsonify({"status": "error", "message": "not connected"}), 400
    data = request.get_json(silent=True) or {}
    cmd = data.get("cmd", "").strip()
    if not cmd:
        return jsonify({"status": "error", "message": "empty cmd"}), 400
    try:
        api.send_cmd(cmd)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/pico_lines")
def pico_lines():
    lines = api.read_pico_lines()
    return jsonify({"lines": lines})


@app.route("/send_message", methods=["POST"])
def send_message():
    """Dispatch a control message to the running scenario."""
    data = request.get_json(silent=True) or {}
    try:
        api.process_message(data)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/tune")
def tune_page():
    return render_template("tune.html")


@app.route("/debug")
def debug_page():
    """Display debugging information."""
    return render_template("debug.html")


@app.route("/manual_params", methods=["GET", "POST"])
def manual_params_route():
    global manual_mode, _manual_params
    if request.method == "GET":
        return jsonify({"manual": manual_mode, **_manual_params})
    data = request.get_json(silent=True) or {}
    param = data.get("param")
    value = data.get("value")
    if param not in _manual_params:
        return jsonify({"status": "error", "message": "bad param"}), 400
    _manual_params[param] = value
    if manual_mode:
        _apply_params(_manual_params)
        frame = api.raw_pipe.get_raw_frame()
        mask = api.mask_pipe.get_masked_frame()
        if frame is not None:
            api.tracker_mgr.force_redetect(frame, mask=mask)
    return jsonify({"status": "ok"})


@app.route("/manual_mode", methods=["POST"])
def manual_mode_route():
    global manual_mode
    data = request.get_json(silent=True) or {}
    enable = bool(data.get("enable"))
    manual_mode = enable
    if manual_mode:
        _apply_params(_manual_params)
    else:
        _reset_defaults()
    frame = api.raw_pipe.get_raw_frame()
    mask = api.mask_pipe.get_masked_frame()
    if frame is not None:
        api.tracker_mgr.force_redetect(frame, mask=mask)
    return jsonify({"status": "ok", "manual": manual_mode})


if __name__ == "__main__":
    try:
        api.start()
        app.run(host="0.0.0.0", port=8000, threaded=True, debug=False, use_reloader=False)
    finally:
        if api.annotated_pipe:
            api.annotated_pipe.stop()
        api.raw_pipe.stop()
        api.mask_pipe.stop()
        api.camera.stop()
