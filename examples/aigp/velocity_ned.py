"""Request -1 m/s north for the first two native race seconds, then zero."""

from miniflight import VelocityNed
from target.aigp.controllers import BaseController
from target.aigp.simulator import AIGPSimulator


class Controller(BaseController[VelocityNed]):
    targets = ("vq1.r1",)

    def update(self, telemetry, frames, race_state) -> VelocityNed | None:
        if race_state is None or not race_state.started:
            return None
        if race_state.time_seconds < 2:
            return VelocityNed(-1, 0, 0)
        if race_state.time_seconds < 3:
            return VelocityNed(0, 0, 0)
        raise StopIteration


if __name__ == "__main__":
    AIGPSimulator(Controller, "vq1.r1").rollout()
