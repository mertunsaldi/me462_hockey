"""Client scenario replicating the built-in StandingBallHitter."""

from ball_example.scenarios import StandingBallHitter, BallReflector


class ClientScenario(BallReflector):
    def __init__(self, plotclock, frame_size):
        super().__init__(plotclock, frame_size)
