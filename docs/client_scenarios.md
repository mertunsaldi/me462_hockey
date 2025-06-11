# Writing Client Scenarios

Clients may upload their own Python modules implementing custom logic. Each file must define a `ClientScenario` class derived from `ball_example.scenarios.Scenario`.

```python
from ball_example.scenarios import Scenario

class ClientScenario(Scenario):
    def update(self, detections):
        # access PlotClock helpers or run your own logic
        pass
```

Use `/load_scenario` to activate the scenario:

```bash
curl -X POST http://localhost:8000/load_scenario -H 'Content-Type: application/json' -d '{"path": "my_scenario.py"}'
```

Alternatively upload the file:

```bash
curl -X POST -F 'file=@my_scenario.py' http://localhost:8000/load_scenario
```

You can send runtime messages to the active scenario with `/send_message`:

```bash
curl -X POST http://localhost:8000/send_message -H 'Content-Type: application/json' -d '{"mode": "stop_defense"}'
```

**Security warning:** the server executes the uploaded Python code directly. Only run scenarios from trusted sources.

## Example: Standing Ball Hitter

The repository contains an example scenario in `examples/standing_hitter_client.py` which simply reuses the built-in `StandingBallHitter` logic.

1. Start the server on the host machine. You can either run the package or execute the script directly:
   ```bash
   # package form (recommended)
   python -m ball_example.app

   # or run the script directly
   python ball_example/app.py
   ```
2. From another terminal (or another machine on the same network) upload the scenario:
   ```bash
   curl -X POST -F 'file=@examples/standing_hitter_client.py' http://localhost:8000/load_scenario
   ```
3. Connect the PlotClock using the **Connect Pico** button (or POST to
   `/connect_pico`).  The button will turn green once the connection succeeds.
4. Open `http://<host-ip>:8000/` in a browser. After uploading the scenario,
   press **Start Scenario** (or POST to `/start_scenario`) to begin execution.
   When a scenario starts, its `on_start()` method runs. The built-in
   `StandingBallHitter` clears any previous calibration so you should see the
   calibration routine whenever you start it.
5. Optionally send messages while it is running:
   ```bash
   curl -X POST http://localhost:8000/send_message -H 'Content-Type: application/json' -d '{"cmd": "stop"}'
   ```
