import cv2, math, threading, time
from flask import Flask, render_template, Response, jsonify, request
from camera import Camera
from trackers import BallTracker
from detectors import ArucoDetector
from renderers import render_overlay, draw_line
from models import Game
from scenarios import *
from ball_example.gadgets import PlotClock

app = Flask(__name__, template_folder='templates', static_folder='static')

# Camera and tracking setup
camera = Camera(src=0)
frame_size = camera.get_resolution()
tracker_mgr = BallTracker()

game = Game()
lock = threading.Lock()

# PlotClock communication setup
plotclock = PlotClock(port=None, baudrate=115200, timeout=0.2)
pico_lock = threading.Lock()
pico_connected = False

# ------------------------------------------------------------------
# Set your active scenario here (None for no scenario)
# Options: None, 'standing', 'calib'
# _current_scenario = StandingBallHitter(plotclock)
_current_scenario = BallReflector(plotclock, frame_size, speed_tol=0.5)
# ------------------------------------------------------------------

# Only run scenario after start is triggered
scenario_enabled = False

@app.route('/start_scenario', methods=['POST'])
def start_scenario():
    global scenario_enabled
    scenario_enabled = True
    return jsonify({'status': 'ok'})


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
            line_points=None,      # handled below
            extra_points=extra_pts,
            extra_labels=extra_labels
        )

        # 5) draw scenario lines (axes or ball-target line)
        if _current_scenario and scenario_enabled and scenario_line:
            # scenario_line may be a single tuple or list of tuples
            lines = scenario_line if isinstance(scenario_line, list) else [scenario_line]
            for idx, (p1, p2) in enumerate(lines):
                # if two axes, color X red, Y blue; otherwise white
                if len(lines) == 2:
                    color = (255, 0, 0) if idx == 0 else (0, 0, 255)
                else:
                    color = (255, 255, 255)
                draw_line(annotated, p1, p2, color=color, thickness=2)

        ok, buf = cv2.imencode('.jpg', annotated)
        if not ok:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stats')
def stats():
    with lock:
        balls = list(game.balls)
        markers = list(game.arucos)
    speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]
    return jsonify({
        'num_balls': len(balls),
        'ball_ids': [b.id for b in balls],
        'speeds': speeds,
        'num_markers': len(markers),
        'marker_ids': [m.id for m in markers],
        'marker_centers': [m.center for m in markers]
    })

@app.route('/connect_pico', methods=['POST'])
def connect_pico():
    global pico_connected
    with pico_lock:
        if pico_connected:
            return jsonify({'status': 'ok'})
        try:
            plotclock.start_comms()
            plotclock.send_command('mode', 4)
            pico_connected = True
            return jsonify({'status': 'ok'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/send_cmd', methods=['POST'])
def send_cmd():
    global pico_connected
    if not pico_connected:
        return jsonify({'status': 'error', 'message': 'not connected'}), 400
    data = request.get_json(silent=True) or {}
    cmd = data.get('cmd', '').strip()
    if not cmd:
        return jsonify({'status': 'error', 'message': 'empty cmd'}), 400
    with pico_lock:
        try:
            plotclock.send_command(cmd)
            return jsonify({'status': 'ok'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

if __name__ == '__main__':
    camera.start()
    app.run(host='0.0.0.0', port=8000, threaded=True, debug=True)