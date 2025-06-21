"""Low level API usage example.

This script shows how to access raw detections and
send direct commands to a PlotClock.
"""

import time
from ball_example.game_api import GameAPI


def main() -> None:
    """Run the low level demo."""
    api = GameAPI()
    api.start()

    while True:
        time.sleep(0.1)
        with api.lock:
            balls = list(api.balls)
            markers = list(api.arucos)
        print("balls", [b.center for b in balls])
        print("markers", [m.id for m in markers])

        if balls and not api.pico_connected:
            try:
                api.connect_pico()
                api.send_cmd("home")
            except Exception as e:
                print("pico error", e)


if __name__ == "__main__":
    main()

