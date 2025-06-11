"""Client scenario replicating the built-in StandingBallHitter."""

from ball_example.app import plotclock, frame_size
from ball_example.scenarios import StandingBallHitter, BallReflector


class ClientScenario(BallReflector):
    def __init__(self):
        super().__init__(plotclock, frame_size)
