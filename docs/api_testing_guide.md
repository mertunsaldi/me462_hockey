# API Testing Guide

This guide describes how to interact with the Flask server from another device on
your local network.

1. **Run the server**

   ```bash
   # package form (recommended)
   python -m ball_example.app

   # or run the script directly
   python ball_example/app.py
   ```
   The server listens on port `8000` by default.

2. **Discover the host IP**

   On the machine running the server, determine its local IP address.  You can
   use `ip addr` on Linux or `ipconfig` on Windows.  Suppose the address is
   `192.168.1.100`.

3. **Connect the PlotClock**

   Use the **Connect Pico** button on the web page or POST to `/connect_pico`.
   Make sure the Arena Manager marker (ID 0) and the PlotClock marker (ID 1) are
   clearly visible to the camera.  Any additional PlotClock markers will be
   ignored during calibration:

   ```bash
   curl -X POST http://192.168.1.100:8000/connect_pico
   ```

4. **Send commands from another device**

   On another computer connected to the same network, you can use `curl` or a
   small Python script to post commands.  Below is an example using `curl`:

   ```bash
   curl -X POST http://192.168.1.100:8000/load_commands \
        -H 'Content-Type: application/json' \
        -d '{"commands": [{"action": "calibrate", "clock": 0}]}'
   ```

   Or using Python:

   ```python
   import requests
   url = 'http://192.168.1.100:8000/load_commands'
   data = {"commands": [{"action": "calibrate", "clock": 0}]}
   r = requests.post(url, json=data)
   print(r.json())
   ```

5. **Load a Python scenario**

   ```bash
   curl -X POST http://192.168.1.100:8000/load_scenario -H 'Content-Type: application/json' \
        -d '{"path": "myscenario.py"}'
   ```

6. **Start the scenario**

   ```bash
   curl -X POST http://192.168.1.100:8000/start_scenario
   ```

   After the scenario is running you can send messages:

   ```bash
   curl -X POST http://192.168.1.100:8000/send_message \
        -H 'Content-Type: application/json' -d '{"cmd": "stop"}'
   ```

7. **Video stream**

Navigate to `http://192.168.1.100:8000/` in a browser to view the live
   camera feed and overlay.

8. **Retrieve debug information**

   ```bash
   curl http://192.168.1.100:8000/debug_data
   ```

   This endpoint exposes all diagnostics from `GameAPI.debug_info`,
   including `ball_details` and image processing parameters.

Only trusted clients should be allowed to issue commands, as they can control the
connected PlotClocks.

## Python helper module

For quick scripting you can import helper functions directly from the project
root:

```python
from high_level import discover_plotclocks, calibrate_clocks
```

These wrappers simplify PlotClock discovery and calibration when using the
`GameAPI` class programmatically.
