"""Client scenario replicating the built-in StandingBallHitter."""

from ball_example.app import plotclock
from ball_example.scenarios import StandingBallHitter


class ClientScenario(StandingBallHitter):
    def __init__(self):
        super().__init__(plotclock)
