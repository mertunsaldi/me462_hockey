import cv2
from flask import Flask, render_template, Response, jsonify
from camera import Camera
from detectors import detect_balls, estimate_velocities
from renderers import render_overlay
from models import Game, Ball
from trackers import TrackerManager
import threading
import time

# Flask app setup
app = Flask(
    __name__,
    template_folder='templates',
    static_folder='static'
)

# Initialize camera, game state, and tracker manager
global camera, game, prev_balls, lock, tracker_mgr, initialized
camera = Camera(src=0)
game = Game()
prev_balls = []
lock = threading.Lock()

# Tracker manager: reinitialize every N frames, persistent IDs
tracker_mgr = TrackerManager(reinit_interval=30, max_match_distance=50.0)
initialized = False


def generate_frames():
    """
    Main loop: capture frame, initialize or update trackers, estimate velocities,
    update game state, annotate, and stream as MJPEG.
    """
    global prev_balls, initialized
    while True:
        grabbed, frame = camera.read()
        if not grabbed or frame is None:
            time.sleep(0.01)
            continue

        if not initialized:
            # First-time: detect and set up trackers
            detections = detect_balls(frame)
            # Match detections to internal state (empty at first)
            tracker_mgr._match_detections(detections)
            # Initialize trackers on these detections
            tracker_mgr._reset_trackers(frame)
            curr_balls = list(tracker_mgr.balls.values())
            initialized = True
        else:
            # Update existing trackers (with periodic re-detection internally)
            curr_balls = tracker_mgr.update(frame)

        # Estimate velocities
        estimate_velocities(prev_balls, curr_balls)
        prev_balls = [Ball(center=b.center, radius=b.radius, color=b.color, ball_id=b.id) for b in curr_balls]

        # Update shared game state
        with lock:
            game.set_balls(curr_balls)

        # Annotate and encode frame
        annotated = render_overlay(frame, curr_balls)
        ret, buffer = cv2.imencode('.jpg', annotated)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
        )

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/stats')
def stats():
    with lock:
        balls = game.balls.copy()
    ids = [b.id for b in balls]
    speeds = [round((b.velocity[0]**2 + b.velocity[1]**2)**0.5, 2) for b in balls]
    return jsonify({
        'num_balls': len(balls),
        'speeds': speeds,
        'ids': ids
    })

if __name__ == '__main__':
    camera.start()
    app.run(host='0.0.0.0', port=8000, threaded=True, debug=True)
