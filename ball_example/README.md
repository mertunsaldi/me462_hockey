# Ball Example

This directory contains a simple Flask application that tracks balls and controls a PlotClock device.

## Running

```bash
python app.py
```

The server listens on `http://localhost:8000` by default.

## API

### `POST /start_scenario`
Enables the currently selected scenario.

### `POST /connect_pico`
Establishes serial communication with the PlotClock.

### `POST /send_cmd`
Sends a raw command string to the PlotClock. Example:

```bash
curl -X POST http://localhost:8000/send_cmd \
     -H 'Content-Type: application/json' \
     -d '{"cmd": "setxy 10 10"}'
```

### `POST /send_message`
Dispatches an arbitrary JSON payload to the active scenario. The scenario must implement
`process_message(data)` to handle the message.

```bash
curl -X POST http://localhost:8000/send_message \
     -H 'Content-Type: application/json' \
     -d '{"type": "greet", "value": "hello"}'
```

If the scenario does not provide a custom response, the endpoint returns:

```json
{"status": "ok"}
```
