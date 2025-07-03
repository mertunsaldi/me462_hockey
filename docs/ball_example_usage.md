# Ball Example Overview

This document explains how to use the `ball_example` module, how to run the provided Flask server, and how you can build your own user interface and scenarios. After reading this guide you should be able to develop a new application around the core API.

## Running the Server

The repository ships with a ready-to-use Flask application located in `ball_example/app.py`. Start it either as a module or by executing the script directly:

```bash
python -m ball_example.app     
# or
python ball_example/app.py
```

By default the server listens on `{your_ip}:8000` and begins processing frames from camera index 0. The web interface served at `/` shows the live video with all detections overlaid. Also, `{your_ip}:8000/debug` shows information about detections and object properties.

Note: `{your_ip}:8000/tune` now displays only the detected balls overlaid on the raw feed.

The server exposes several endpoints used by the JavaScript front end (see `templates/` and `static/`). The most important ones are:

- `POST /connect_pico` – scan for PlotClock devices and build the default scenario when marker 0 and 1 are present.
- `POST /start_scenario` – activate whichever scenario was previously loaded.
- `POST /stop_scenario` – stop the running scenario.
- `POST /load_commands` – load a list of high level commands.
- `POST /load_scenario` – load a Python file defining a `ClientScenario` (see below).
- `POST /send_message` – send an ad-hoc message to the active scenario.
- `GET /debug_data` – retrieve extended diagnostics returned by `GameAPI.debug_info()`.

Refer to `docs/api_testing_guide.md` for examples of how to call these endpoints from another machine.

### Example Workflow

1. Flash `PlotClockMaster.py` to the master Pico and the appropriate
   firmware (`PlotClockSlave.py` or `arena_manager_slave.py`) to each
   device.
2. Connect the master Pico to your PC via USB and run the server:

   ```bash
   python -m ball_example.app
   ```
3. Use the web UI or POST to `/connect_pico` to detect the attached
   devices.
4. Upload a command list with `/load_commands` or a custom scenario with
   `/load_scenario`.
5. Start execution via `/start_scenario` and open
   `http://<host-ip>:8000/` in your browser to view the stream and
   overlays.

## Key Classes

### `GameAPI`

`GameAPI` owns the camera, tracking pipelines and the currently active scenario. The Flask app only acts as a thin wrapper around this class. Important methods include:

- `start()` – start the camera and pipelines.
- `set_cam_source(src, width, height, fourcc)` – switch the camera source.
- `connect_pico()` – open the serial connection to the Pico master and detect PlotClocks.
- `load_commands(commands)` – create a `CommandScenario` from a list of dictionaries.
- `load_scenario(path)` – load a Python module that defines `ClientScenario`.
- `start_scenario()` / `stop_scenario()` – control the currently loaded scenario.
- `start_clock_mode(clock, mode, target_mm)` – start an action like `attack`, `defend` or `hit_standing` for a single PlotClock.
- `start_move_object(manager, obj, target_mm)` – create a `MoveObject` scenario to move a ball or obstacle.
- `debug_info()` – return dictionaries describing all detections and parameters.

A GUI can simply instantiate `GameAPI`, call `start()` and then display `get_annotated_frame()` in a loop.

### `PlotClock` and `ArenaManager`

Both classes live in `ball_example/gadgets.py` and encapsulate communication with the robot hardware. `PlotClock` controls a hitting arm while `ArenaManager` can also grip and move objects. Their `calibrate()` methods expect lists of ArUco detections and drive the internal calibration FSM.

Useful helpers are:

- `send_command(cmd)` – send raw firmware commands.
- `pixel_to_mm()` / `mm_to_pixel()` – convert between image pixels and robot coordinates once calibrated.
- `draw_working_area(frame)` – draw the reachable polygon for visualisation.
- Behaviour helpers such as `attack()`, `defend()` and `hit_standing_ball()` which construct corresponding scenarios.

### Scenarios

Every scenario derives from the base class `ball_example.scenarios.Scenario` which defines the lifecycle hooks:

```python
class Scenario:
    finished: bool = False
    def on_start(self):
        pass
    def on_stop(self):
        pass
    def process_message(self, message: dict):
        pass
    def update(self, detections):
        raise NotImplementedError
```

Built-in scenarios in `scenarios.py` implement more complex behaviours. Notable ones are:

- `MoveBallHitRandom` – moves the ball to the hitter and then strikes it toward a random target.
- `StandingBallHitter` and `SingleHitStandingBallHitter` – wait for a ball to stop then hit it toward a provided point.
- `BallReflector` – defend the goal by reflecting incoming balls.
- `MoveObject` – used by `ArenaManager` to grab and place objects.

All scenarios provide overlay callbacks (`get_line_points`, `get_extra_points`, `get_extra_labels`) used by the web UI for drawing helper graphics.

## Writing Your Own Scenario

Create a new Python file defining a subclass named `ClientScenario`. You can import any helpers from `ball_example`:

```python
from ball_example.scenarios import Scenario
from ball_example.gadgets import PlotClock

class ClientScenario(Scenario):
    def __init__(self, clock: PlotClock, frame_size):
        self.clock = clock
        self.frame_size = frame_size

    def update(self, detections):
        # Implement your frame logic here
        pass
```

Load the file at runtime using:

```bash
curl -X POST http://localhost:8000/load_scenario -F 'file=@myscenario.py'
```

Then start it via `/start_scenario`.

For inspiration check the implementation of `MoveBallHitRandom` in `ball_example/scenarios.py`.

## Building a Custom GUI

`app.py` demonstrates how to wire `GameAPI` into a Flask web interface. A user can develop their own GUI by creating a small program that imports `GameAPI` and periodically fetches frames. The minimal loop looks like:

```python
from ball_example.game_api import GameAPI

api = GameAPI()
api.start()

while True:
    frame = api.get_annotated_frame()
    if frame is not None:
        # show frame using cv2.imshow or any GUI toolkit
        pass
```

You are free to design a completely different interface, e.g. using `tkinter` or `PyQt`. Just call the methods on `GameAPI` to move the PlotClocks and trigger scenarios as needed. One may take a look at demo files under `/examples` folder, they provide simple features using tkinter.

## Firmware and Calibration Scripts

The hardware is driven by a pair of Raspberry Pi Pico boards. The repository contains MicroPython firmware for these devices inside the `PlotClock` directory:

- `PlotClockMaster.py` – runs on the **master** Pico. It simply forwards all serial commands from the host to the slave board via UART and echoes back any replies. Flash this file onto the Pico connected to the host PC.
- `PlotClockSlave.py` – firmware for a PlotClock robot. It decodes commands such as `p.getMinY()` or `p.setXY()` and sends a script back or drives the servos accordingly. Deploy it to the Pico attached to the hitter plotclock.
- `arena_manager_slave.py` – similar to the PlotClock slave but controls the **Arena Manager** with the gripper. Flash this on the Pico driving the manager mechanism.

Use a tool like `mpremote` or `rshell` or `Thonny` to copy the scripts to the respective boards, naming them `main.py`. The master Pico appears as a USB serial device on the host machine while the slave Pico is connected only to the master via UART.

Calibration of the arena manager is performed with `calibration_map.py`. Execute
this script on the host after connecting the devices. It builds a grid of points
inside the working area, moves the manager to each one and records the resulting
image coordinates. Once complete it writes `calibration_map.csv` and
`calibration_poly.csv` which are loaded automatically by `ArenaManager`. Optionally, one may also test how the calibration turned out via `Go to Corrected Points` button.

Note: It is necessary for the camera to see all the arena corners (4x4 Aruco markers of id 13).

### Calibration Logic

`PlotClock` and `ArenaManager` share a simple calibration procedure used to
translate between image pixels and millimetres. Each `PlotClock` moves to three
pre-defined points within its workspace. After every move the ArUco marker on
the hitter is detected and the pixel centre stored. Once all three positions are
captured the unit vectors `u_x` and `u_y` describing the x and y axes are
computed. The calibration also determines the pixel origin so the following
relation holds:

```text
pixel = origin + u_x * x_mm + u_y * y_mm
```

`mm_to_pixel()` and `pixel_to_mm()` simply evaluate this expression (or its
inverse) to convert coordinates.

For the arena manager the orientation is derived from two servo markers
(`id 47`) and the manager marker (`id 0`). The helper function
`compute_servo_transform()` in `calibration_map.py` calculates the same
`origin`, `u_x` and `u_y` vectors from those three detections before the regular
calibration routine begins.

`calibration_map.py` then commands the manager through a series of grid points
covering the allowed polygon. For each point the measured pixel location is
converted back to millimetres and stored in `calibration_map.csv`. The script
fits a 2D quadratic polynomial that maps measured positions to commanded targets.
Its coefficients are written to `calibration_poly.csv` and loaded automatically
by `ArenaManager` for future corrections.

## Putting It All Together

Below is a more detailed outline of the typical order in which `GameAPI` methods are called when building an application similar to `app.py`.  The comments show which calls are performed once during setup and which run repeatedly inside the main loop:

```python
from ball_example.game_api import GameAPI

api = GameAPI()
api.set_cam_source(0, width=1280, height=720, fourcc="MJPG")
api.start()                 # start camera and tracking        (one time)
api.connect_pico()          # open the serial link to the master Pico  (one time)

# Either load a list of commands or a custom scenario
# api.load_commands([...])
api.load_scenario("myscenario.py")

api.start_scenario()        # start running scenario           (one time)

try:
    while True:                            # main loop runs indefinitely
        frame = api.get_annotated_frame()
        if frame is not None:
            display(frame)  # e.g. cv2.imshow or any GUI toolkit
        # optionally send api.send_message(), api.debug_info(), ...
        # repeated logic goes here
except KeyboardInterrupt:
    pass
finally:
    api.stop_scenario()     # once, when exiting
```

This structure lets you build GUIs in any framework. All high level behaviour is exposed through `GameAPI`.

## Other Important Modules

The project contains several helper modules that can simplify
development:

- `high_level.py` – convenience functions such as
  `discover_plotclocks`, `calibrate_clocks` and `draw_arena` which are
  useful when writing small scripts.
- `simple_api.py` – interprets a list of action dictionaries and runs
  them sequentially using the `CommandScenario` class.
- `detectors.py` and `trackers.py` – implement ArUco and ball detection
  along with Kalman-based tracking.

## Next Steps

- Explore `docs/api_testing_guide.md` for REST examples.
- Read `docs/client_scenarios.md` for more details on uploading scenarios.
- Examine the HTML templates under `ball_example/templates/` to see how the existing UI interacts with the endpoints.

Using these pieces you should be able to extend the system and build your own applications around the ball example.

