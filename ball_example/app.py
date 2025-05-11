import cv2, math, threading, time
from flask import Flask, render_template, Response, jsonify
from camera import Camera
from trackers import BallTracker
from detectors import ArucoDetector        # ← import your Aruco detector
from renderers import render_overlay
from models import Game

app = Flask(__name__, template_folder='templates', static_folder='static')

camera     = Camera(src=0)
tracker_mgr= BallTracker()                # thresholds in trackers.py
game       = Game()
lock       = threading.Lock()             # for stats JSON

def generate_frames():
    while True:
        grabbed, frame = camera.read()
        if not grabbed:
            time.sleep(0.01)
            continue

        # 1) detect balls
        balls = tracker_mgr.update(frame)

        # 2) detect arucos
        arucos = ArucoDetector.detect(frame)

        # 3) update shared game state
        with lock:
            game.set_balls(balls)
            game.set_arucos(arucos)

        # 4) render both
        annotated = render_overlay(frame, balls, arucos)

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
        balls  = game.balls.copy()
        arucos = game.arucos.copy()
    speeds = [round(math.hypot(*map(float, b.velocity)), 2) for b in balls]

    # return both ball and marker info
    return jsonify({
        'num_balls': len(balls),
        'ball_ids':  [b.id for b in balls],
        'speeds':    speeds,
        'num_markers': len(arucos),
        'marker_ids': [m.id for m in arucos],
        'marker_centers': [m.center for m in arucos]
    })

if __name__ == '__main__':
    camera.start()
    app.run(host='0.0.0.0', port=8000, threaded=True, debug=True)