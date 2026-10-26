"""Request -1 m/s north for two seconds, then observe speed below 0.1 m/s."""

import math

from miniflight import State, VelocityNed


class Program:
    def __init__(self):
        self.started_at = self.braking_time = None

    def update(self, state: State | None) -> VelocityNed | None:
        feedback = None if state is None else state.rate_feedback
        if feedback is None or state.motion is None:
            return None
        if self.started_at is None:
            self.started_at = feedback.time
        elapsed = feedback.time - self.started_at
        if elapsed < 2:
            return VelocityNed(-1, 0, 0)
        if elapsed >= 5:
            raise TimeoutError("velocity did not settle within five device seconds")
        if self.braking_time is None:
            self.braking_time = state.motion.time
        elif state.motion.time > self.braking_time and math.hypot(*state.motion.velocity) < .1:
            raise StopIteration
        return VelocityNed(0, 0, 0)
