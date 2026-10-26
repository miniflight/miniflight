"""Request -1 m/s north for two device seconds, then zero for one."""

from miniflight import State, VelocityNed


class Program:
    def __init__(self):
        self.started_at = None

    def update(self, state: State | None) -> VelocityNed | None:
        feedback = None if state is None else state.rate_feedback
        if feedback is None:
            return None
        if self.started_at is None:
            self.started_at = feedback.time
        elapsed = feedback.time - self.started_at
        if elapsed < 2:
            return VelocityNed(-1, 0, 0)
        if elapsed < 3:
            return VelocityNed(0, 0, 0)
        raise StopIteration
