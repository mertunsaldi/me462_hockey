#!/usr/bin/env python3
"""
High-FPS ball-tracking server for Raspberry Pi or USB cam
─────────────────────────────────────────────────────────
• Full-resolution capture (1280×720 default)
• Overlays rendered server-side (no lag)
• TurboJPEG hardware-optimised compression (~7 ms per frame vs 22 ms)
• Three-stage pipeline keeps UI at rock-solid 30 fps
"""

import os, math, time, threading, cv2
from collections import deque
from io import BytesIO
from flask import Flask, render_template, Response, jsonify, request

# ─── Try TurboJPEG ──────────────────────────────────────────────────────────
try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_444
    jpeg_encoder = TurboJPEG()
    def encode_jpeg(img, quality=80):
        return jpeg_encoder.encode(img,
                                   pixel_format=TJPF_BGR,
                                   jpeg_subsample=TJSAMP_444,
                                   quality=quality)
    print("[JPEG] Using TurboJPEG hardware encoder")
except Exception as e:
    print(f"[JPEG] TurboJPEG unavailable ({e!s}); falling back to cv2.imencode")
    def encode_jpeg(img, quality=80):
        ok, buf = cv2.imencode(".jpg", img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()

# ─── Local modules (your own code) ──────────────────────────────────────────
from camera   import USBCamera, PicameraCamera   # (unchanged camera.py)
from trackers import BallTracker
from detectors import ArucoDetector
from renderers import render_overlay, draw_line
from models    import Game
from scenarios import StandingBallHitter
from gadgets   import PlotClock
# ----------------------------------------------------------------------------

app = Flask(__name__, template_folder="templates", static_folder="static")

# ─── Camera selection ───────────────────────────────────────────────────────
USE_PICAM = os.environ.get("CAMERA_TYPE", "picam").lower() == "picam"
camera    = PicameraCamera(main_size=(1280, 720), fps=30) if USE_PICAM else \
            USBCamera(src=0, width=1280, height=720, fps=30)

# ─── CV / game state ────────────────────────────────────────────────────────
tracker_mgr = BallTracker()
game        = Game()
game_lock   = threading.Lock()

_current_scenario = StandingBallHitter(None)
scenario_enabled  = False

# ─── Optional PlotClock / Pico comms ────────────────────────────────────────
plotclock      = PlotClock(port=None, baudrate=115200, timeout=0.2)
pico_lock      = threading.Lock()
pico_connected = False

# ─── Shared data between threads ────────────────────────────────────────────
detect_lock       = threading.Lock()
latest_balls      = []
latest_markers    = []

annotated_q = deque(maxlen=1)   # detect → encode
jpeg_q      = deque(maxlen=1)   # encode → stream

# ════════════════════════════════════════════════════════════════════════════
# 1.  Detection + drawing thread                                             ║
# ════════════════════════════════════════════════════════════════════════════
def detection_loop(target_fps=30):
    period = 1.0 / target_fps
    while True:
        start = time.time()
        src = camera.latest()              # newest full-res frame (no copy)
        if src is None:
            time.sleep(0.004)
            continue

        frame = src.copy()                 # working copy for overlays

        # ── heavy CV ───────────────────────────────────────────────────────
        balls   = tracker_mgr.update(frame)
        markers = ArucoDetector.detect(frame)

        extra_pts = extra_labels = scenario_line = None
        if _current_scenario and scenario_enabled:
            _current_scenario.update(balls + markers)
            scenario_line = _current_scenario.get_line_points()
            extra_pts     = _current_scenario.get_extra_points()
            extra_labels  = _current_scenario.get_extra_labels()

        # ── overlay drawing ────────────────────────────────────────────────
        annotated = render_overlay(
            frame, balls, markers,
            line_points=None,
            extra_points=extra_pts,
            extra_labels=extra_labels,
        )

        if scenario_line and scenario_enabled:
            lines = scenario_line if isinstance(scenario_line, list) else [scenario_line]
            for idx, (p1, p2) in enumerate(lines):
                color = (255, 0, 0) if len(lines) == 2 and idx == 0 else \
                        (0, 0, 255)  if len(lines) == 2 else (255, 255, 255)
                draw_line(annotated, p1, p2, color, 2)

        # ── publish to encoder queue ───────────────────────────────────────
        annotated_q.append(annotated)

        with detect_lock:
            latest_balls[:]   = balls
            latest_markers[:] = markers
        with game_lock:
            game.set_balls(balls)
            game.set_arucos(markers)

        # optional frame-rate clamp
        dt = time.time() - start
        if dt < period:
            time.sleep(period - dt)

# ════════════════════════════════════════════════════════════════════════════
# 2.  JPEG encoder thread (TurboJPEG)                                        ║
# ════════════════════════════════════════════════════════════════════════════
def encoder_loop():
    while True:
        if not annotated_q:
            time.sleep(0.003)
            continue
        frame = annotated_q.pop()
        try:
            jpg = encode_jpeg(frame, quality=80)
            jpeg_q.append(jpg)
        except Exception as e:
            print("JPEG encode error:", e)

# ════════════════════════════════════════════════════════════════════════════
# 3.  MJPEG generator for Flask                                              ║
# ════════════════════════════════════════════════════════════════════════════
def generate_frames():
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n%s\r\n"
    while True:
        if not jpeg_q:
            time.sleep(0.01)
            continue
        yield boundary % jpeg_q.pop()

# ─── Flask routes ───────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/stats")
def stats():
    with game_lock:
        balls   = list(game.balls)
        markers = list(game.arucos)
    speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]
    return jsonify({
        "num_balls":      len(balls),
        "ball_ids":       [b.id for b in balls],
        "speeds":         speeds,
        "num_markers":    len(markers),
        "marker_ids":     [m.id for m in markers],
        "marker_centers": [m.center for m in markers],
    })

@app.route("/start_scenario", methods=["POST"])
def start_scenario():
    global scenario_enabled
    scenario_enabled = True
    return jsonify({"status": "ok"})

# ── Pico comms endpoints (unchanged) ────────────────────────────────────────
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
    cmd  = data.get("cmd", "").strip()
    if not cmd:
        return jsonify({"status": "error", "message": "empty cmd"}), 400
    with pico_lock:
        try:
            plotclock.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

# ─── Main entry point ───────────────────────────────────────────────────────
if __name__ == "__main__":
    camera.start()
    threading.Thread(target=detection_loop, daemon=True).start()
    threading.Thread(target=encoder_loop,   daemon=True).start()
    try:
        app.run(host="0.0.0.0", port=8000, threaded=True, debug=False)
    finally:
        camera.stop()