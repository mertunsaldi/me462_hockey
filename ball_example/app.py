import cv2, math, threading, time
from flask import Flask, render_template, Response, jsonify
from camera import Camera
from trackers import BallTracker
from detectors import ArucoDetector
from renderers import render_overlay
from models import Game
from scenarios import StandingBallHitter

app = Flask(__name__, template_folder='templates', static_folder='static')

camera      = Camera(src=0)
tracker_mgr = BallTracker()
game        = Game()
lock        = threading.Lock()

# ------------------------------------------------------------------
# Set your active scenario here (None for no scenario)
# Options: None, 'standing'
ACTIVE_SCENARIO_NAME = 'standing'
_SCENARIO_MAP = {
    'standing': StandingBallHitter,
    None: None
}
_current_scenario = _SCENARIO_MAP.get(ACTIVE_SCENARIO_NAME, None)
if _current_scenario:
    _current_scenario = _current_scenario()
# ------------------------------------------------------------------

def generate_frames():
    while True:
        grabbed, frame = camera.read()
        if not grabbed:
            time.sleep(0.01)
            continue

        # 1) detect objects
        balls   = tracker_mgr.update(frame)
        markers = ArucoDetector.detect(frame)

        # 2) update game state
        with lock:
            game.set_balls(balls)
            game.set_arucos(markers)

        # 3) run scenario logic (if any)
        line_pts     = None
        extra_pts    = None
        extra_labels = None
        if _current_scenario:
            detections = []
            detections.extend(balls)
            detections.extend(markers)
            _current_scenario.update(detections)
            line_pts     = _current_scenario.get_line_points()
            extra_pts    = _current_scenario.get_extra_points()
            extra_labels = _current_scenario.get_extra_labels()

        # 4) render all overlays
        annotated = render_overlay(
            frame,
            balls,
            markers,
            line_points=line_pts,
            extra_points=extra_pts,
            extra_labels=extra_labels
        )

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
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stats')
def stats():
    with lock:
        balls   = list(game.balls)
        markers = list(game.arucos)
    speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]

    return jsonify({
        'num_balls':     len(balls),
        'ball_ids':      [b.id for b in balls],
        'speeds':        speeds,
        'num_markers':   len(markers),
        'marker_ids':    [m.id for m in markers],
        'marker_centers':[m.center for m in markers]
    })

if __name__ == '__main__':
    camera.start()
    app.run(host='0.0.0.0', port=8000, threaded=True, debug=True)