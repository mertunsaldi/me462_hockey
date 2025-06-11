import cv2, math, threading
from flask import Flask, render_template, Response, jsonify, request
from camera import Camera
from trackers import BallTracker
from detectors import ArucoDetector
from renderers import render_overlay, draw_line
from models import Game
from scenarios import *
from ball_example.gadgets import PlotClock
from simple_api import CommandScenario, sort_plotclocks
from scenario_loader import load_scenario, ScenarioLoadError

app = Flask(__name__, template_folder="templates", static_folder="static")

import logging

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

# Camera and tracking setup
camera = Camera(src=0)
frame_size = camera.get_resolution()
tracker_mgr = BallTracker()

game = Game()
lock = threading.Lock()

# PlotClock communication setup
plotclock = PlotClock(port=None, baudrate=115200, timeout=0.2)
plotclocks = [plotclock]
pico_lock = threading.Lock()
pico_connected = False

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


def generate_frames():
    while True:
        grabbed, frame = camera.read()
        if not grabbed:
            time.sleep(0.01)
            continue

        # 1) detect objects
        balls = tracker_mgr.update(frame)
        markers = ArucoDetector.detect(frame)

        # 2) update game state
        with lock:
            game.set_balls(balls)
            game.set_arucos(markers)

        # 3) run scenario logic (if any and enabled)
        scenario_line = None
        extra_pts = None
        extra_labels = None
        if _current_scenario and scenario_enabled:
            detections = balls + markers
            _current_scenario.update(detections)
            scenario_line = _current_scenario.get_line_points()
            extra_pts = _current_scenario.get_extra_points()
            extra_labels = _current_scenario.get_extra_labels()

        # 4) render overlays for balls/markers and extra points
        annotated = render_overlay(
            frame,
            balls,
            markers,
            line_points=None,  # handled below
            extra_points=extra_pts,
            extra_labels=extra_labels,
        )

        # 5) draw scenario lines (axes or ball-target line)
        if _current_scenario and scenario_enabled and scenario_line:
            # scenario_line may be a single tuple or list of tuples
            lines = (
                scenario_line if isinstance(scenario_line, list) else [scenario_line]
            )
            for idx, (p1, p2) in enumerate(lines):
                # if two axes, color X red, Y blue; otherwise white
                if len(lines) == 2:
                    color = (255, 0, 0) if idx == 0 else (0, 0, 255)
                else:
                    color = (255, 255, 255)
                draw_line(annotated, p1, p2, color=color, thickness=2)

        ok, buf = cv2.imencode(".jpg", annotated)
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


if __name__ == "__main__":
    try:
        camera.start()
        app.run(host="0.0.0.0", port=8000, threaded=True, debug=False, use_reloader=False)
    finally:
        camera.stop()
