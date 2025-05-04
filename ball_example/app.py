import cv2, math, threading, time
from flask import Flask, render_template, Response, jsonify
from camera import Camera
from trackers import TrackerManager
from renderers import render_overlay
from models import Game

app = Flask(__name__, template_folder='templates', static_folder='static')

camera = Camera(src=0)
tracker_mgr = TrackerManager()          # no params – thresholds in trackers.py
game = Game()
lock = threading.Lock()                 # for stats JSON

def generate_frames():
    while True:
        grabbed, frame = camera.read()
        if not grabbed:
            time.sleep(0.01)
            continue

        balls = tracker_mgr.update(frame)     # ← single call
        with lock:
            game.set_balls(balls)

        annotated = render_overlay(frame, balls)
        ok, buf = cv2.imencode('.jpg', annotated)
        if not ok:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stats')
def stats():
    with lock:
        balls = game.balls.copy()
    speeds = [round((float(b.velocity[0])**2 + float(b.velocity[1])**2) ** 0.5, 2)
              for b in balls]
    return jsonify({'num_balls': len(balls),
                    'ids': [b.id for b in balls],
                    'speeds': speeds})

if __name__ == '__main__':
    camera.start()
    app.run(host='0.0.0.0', port=8000, threaded=True, debug=True)