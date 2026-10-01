"""Request -1 m/s on the NED north axis for two seconds, then zero velocity."""

from miniflight import VelocityNed
from target.aigp.controllers import BaseController
from target.aigp.simulator import AIGPSimulator


class Controller(BaseController[VelocityNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.started_at = None

    def update(self, state, gate_index, gates) -> VelocityNed:
        if self.started_at is None:
            self.started_at = state.time
        elapsed = state.time - self.started_at
        if elapsed < 2:
            return VelocityNed(-1, 0, 0)
        if elapsed < 3:
            return VelocityNed(0, 0, 0)
        raise StopIteration


if __name__ == "__main__":
    AIGPSimulator(Controller(), "vq1.r1").rollout()
