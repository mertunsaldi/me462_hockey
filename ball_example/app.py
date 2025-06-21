import cv2, math, threading, time
import numpy as np
from typing import Optional
from flask import Flask, render_template, Response, jsonify, request
from camera import Camera
from trackers import BallTracker, DETECTION_SCALE
from detectors import ArucoDetector
from detectors import BallDetector
from pipelines import RawImagePipeline, MaskedImagePipeline, AnnotatedImagePipeline
from renderers import render_overlay, draw_line
from models import Game
from scenarios import *
from gadgets import PlotClock
from simple_api import CommandScenario, sort_plotclocks
from scenario_loader import load_scenario, ScenarioLoadError

app = Flask(__name__, template_folder="templates", static_folder="static")

#import logging

#log = logging.getLogger("werkzeug")
#log.setLevel(logging.ERROR)

# Camera and tracking setup
# Use MJPEG at 1280x720 for smoother FPS on Linux
camera = Camera(src=2, width=1280, height=720, fourcc="MJPG")
frame_size = camera.get_resolution()
tracker_mgr = BallTracker()
raw_pipe = RawImagePipeline(camera)
mask_pipe = MaskedImagePipeline(raw_pipe, scale=DETECTION_SCALE)

annotated_pipe: Optional[AnnotatedImagePipeline] = None

game = Game()
lock = threading.Lock()

# PlotClock communication setup
plotclock = PlotClock(port=None, baudrate=115200, timeout=0.2)
plotclocks = [plotclock]
pico_lock = threading.Lock()
pico_connected = False

_cam_started = False
if hasattr(app, "before_first_request"):
    @app.before_first_request
    def _ensure_camera_running():
        global _cam_started
        global annotated_pipe
        if not _cam_started and not camera.running:
            camera.start()
            raw_pipe.start()
            mask_pipe.start()
            annotated_pipe = AnnotatedImagePipeline(raw_pipe, _process_annotated)
            annotated_pipe.start()
            _cam_started = True
else:
    @app.before_request
    def _ensure_camera_running():
        global _cam_started
        global annotated_pipe
        if not _cam_started and not camera.running:
            camera.start()
            raw_pipe.start()
            mask_pipe.start()
            annotated_pipe = AnnotatedImagePipeline(raw_pipe, _process_annotated)
            annotated_pipe.start()
            _cam_started = True
        
# Manual tuning state
manual_mode = False
_default_params = {
    "circ": BallDetector.CIRCULARITY_THRESHOLD,
    "area_ratio": BallDetector.AREA_RATIO_THRESHOLD,
    "solidity": BallDetector.SOLIDITY_THRESHOLD,
    "edge_density": BallDetector.EDGE_DENSITY_THRESHOLD,
    "blur": BallDetector.BLUR_KERNEL,
    "sigma": BallDetector.BLUR_SIGMA,
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
    BallDetector.HSV_LOWER = np.array([params["h_low"], params["s_low"], params["v_low"]], dtype=np.uint8)
    BallDetector.HSV_UPPER = np.array([params["h_up"], params["s_up"], params["v_up"]], dtype=np.uint8)

def _reset_defaults():
    global _manual_params
    _manual_params = _default_params.copy()
    _apply_params(_default_params)


# ------------------------------------------------------------------
# Set your active scenario here (None for no scenario)
# Options: None, 'standing', 'calib'
# _current_scenario = StandingBallHitter(plotclock)
# The currently loaded scenario instance, if any
_current_scenario = None
# ------------------------------------------------------------------

# Only run scenario after start is triggered
scenario_enabled = False


@app.route("/start_scenario", methods=["POST"])
def start_scenario():
    """Activate the currently loaded scenario."""
    global scenario_enabled
    if _current_scenario is None:
        return jsonify({"status": "error", "message": "no scenario loaded"}), 400

    if not scenario_enabled:
        try:
            _current_scenario.on_start()
        finally:
            scenario_enabled = True

    return jsonify({"status": "ok"})


@app.route("/load_commands", methods=["POST"])
def load_commands():
    global _current_scenario, scenario_enabled
    data = request.get_json(silent=True) or {}
    commands = data.get("commands")
    if not isinstance(commands, list):
        return jsonify({"status": "error", "message": "commands must be a list"}), 400
    try:
        ordered = sort_plotclocks(plotclocks)
        _current_scenario = CommandScenario(ordered, frame_size, commands)
        scenario_enabled = False
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/load_scenario", methods=["POST"])
def load_scenario_route():
    """Load a custom scenario from a Python file path or uploaded file."""
    global _current_scenario, scenario_enabled
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
        _current_scenario = load_scenario(path, plotclock, frame_size)
        scenario_enabled = False
        return jsonify({"status": "ok"})
    except ScenarioLoadError as e:
        return jsonify({"status": "error", "message": str(e)}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


def _process_annotated(frame):
    mask = mask_pipe.get_masked_frame()

    balls = tracker_mgr.update(frame, mask=mask)
    markers = ArucoDetector.detect(frame)

    with lock:
        game.set_balls(balls)
        game.set_arucos(markers)

    scenario_line = None
    extra_pts = None
    extra_labels = None
    if _current_scenario and scenario_enabled:
        detections = balls + markers
        _current_scenario.update(detections)
        scenario_line = _current_scenario.get_line_points()
        extra_pts = _current_scenario.get_extra_points()
        extra_labels = _current_scenario.get_extra_labels()

    annotated = render_overlay(
        frame,
        balls,
        markers,
        line_points=None,
        extra_points=extra_pts,
        extra_labels=extra_labels,
    )

    if _current_scenario and scenario_enabled and scenario_line:
        lines = (
            scenario_line if isinstance(scenario_line, list) else [scenario_line]
        )
        for idx, (p1, p2) in enumerate(lines):
            if len(lines) == 2:
                color = (255, 0, 0) if idx == 0 else (0, 0, 255)
            else:
                color = (255, 255, 255)
            draw_line(annotated, p1, p2, color=color, thickness=2)

    return annotated


def generate_frames():
    while True:
        if annotated_pipe is None:
            time.sleep(0.01)
            continue
        annotated = annotated_pipe.get_annotated_frame()
        if annotated is None:
            time.sleep(0.01)
            continue

        ok, buf = cv2.imencode(".jpg", annotated)
        if not ok:
            continue

        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
        )


def generate_processed_frames():
    while True:
        mask = mask_pipe.get_masked_frame()
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


@app.route("/stats")
def stats():
    with lock:
        balls = list(game.balls)
        markers = list(game.arucos)
    speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]
    scenario_name = _current_scenario.__class__.__name__ if _current_scenario else None
    return jsonify(
        {
            "num_balls": len(balls),
            "ball_ids": [b.id for b in balls],
            "speeds": speeds,
            "num_markers": len(markers),
            "marker_ids": [m.id for m in markers],
            "marker_centers": [m.center for m in markers],
            "scenario_loaded": _current_scenario is not None,
            "scenario_running": _current_scenario is not None and scenario_enabled,
            "scenario_name": scenario_name,
        }
    )


@app.route("/connect_pico", methods=["POST"])
def connect_pico():
    global pico_connected
    with pico_lock:
        if pico_connected:
            return jsonify({"status": "ok"})
        try:
            plotclock.start_comms()
            plotclock.send_command("mode", 4)
            pico_connected = True
            return jsonify({"status": "ok"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/send_cmd", methods=["POST"])
def send_cmd():
    global pico_connected
    if not pico_connected:
        return jsonify({"status": "error", "message": "not connected"}), 400
    data = request.get_json(silent=True) or {}
    cmd = data.get("cmd", "").strip()
    if not cmd:
        return jsonify({"status": "error", "message": "empty cmd"}), 400
    with pico_lock:
        try:
            plotclock.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/send_message", methods=["POST"])
def send_message():
    """Dispatch a control message to the running scenario."""
    if not _current_scenario:
        return jsonify({"status": "error", "message": "no scenario"}), 400
    data = request.get_json(silent=True) or {}
    try:
        _current_scenario.process_message(data)
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/tune")
def tune_page():
    return render_template("tune.html")


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
        frame = raw_pipe.get_raw_frame()
        mask = mask_pipe.get_masked_frame()
        if frame is not None:
            tracker_mgr.force_redetect(frame, mask=mask)
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
    frame = raw_pipe.get_raw_frame()
    mask = mask_pipe.get_masked_frame()
    if frame is not None:
        tracker_mgr.force_redetect(frame, mask=mask)
    return jsonify({"status": "ok", "manual": manual_mode})


if __name__ == "__main__":
    try:
        camera.start()
        raw_pipe.start()
        mask_pipe.start()
        annotated_pipe = AnnotatedImagePipeline(raw_pipe, _process_annotated)
        annotated_pipe.start()
        app.run(host="0.0.0.0", port=8000, threaded=True, debug=False, use_reloader=False)
    finally:
        raw_pipe.stop()
        mask_pipe.stop()
        if annotated_pipe:
            annotated_pipe.stop()
        camera.stop()