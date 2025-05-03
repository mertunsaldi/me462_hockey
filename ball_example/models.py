from typing import Tuple, Optional, List
import uuid


class Ball:
    """
    Represents a detected ball in the frame.

    Attributes:
        id: Unique identifier for tracking across frames.
        center: (x, y) pixel coordinates of the ball's center.
        radius: Radius of the ball in pixels.
        color: (B, G, R) color used for rendering (optional).
        velocity: (vx, vy) displacement per frame (initialized to zero).
    """
    def __init__(
        self,
        center: Tuple[int, int],
        radius: int,
        color: Optional[Tuple[int, int, int]] = None,
        ball_id: Optional[str] = None
    ):
        self.id: str = ball_id or str(uuid.uuid4())
        self.center: Tuple[int, int] = center
        self.radius: int = radius
        self.color: Optional[Tuple[int, int, int]] = color
        # Velocity in pixels per frame: (dx, dy)
        self.velocity: Tuple[float, float] = (0.0, 0.0)

    def __repr__(self) -> str:
        return (
            f"<Ball id={self.id[:8]} center={self.center} "
            f"radius={self.radius} velocity={self.velocity}>"
        )


class Game:
    """
    Maintains the current state of the game, specifically the list of balls.

    Responsibilities:
        - Store and update the list of Ball objects.
        - Provide utility methods for managing balls.
    """
    def __init__(self):
        # List of currently tracked balls
        self.balls: List[Ball] = []

    def set_balls(self, balls: List[Ball]) -> None:
        """
        Replace the current list of balls with a new list.
        """
        self.balls = balls

    def clear_balls(self) -> None:
        """
        Remove all balls from the game.
        """
        self.balls.clear()

    def add_ball(self, ball: Ball) -> None:
        """
        Add a single Ball to the game state.
        """
        self.balls.append(ball)

    def __iter__(self):
        return iter(self.balls)

    def __len__(self) -> int:
        return len(self.balls)

    def __repr__(self) -> str:
        return f"<Game balls={len(self.balls)}>"
