"""Start the target-independent velocity program after native GO."""

from examples.velocity_ned import Program
from miniflight import VelocityNed, vehicle
from target.aigp.controllers import BaseController
from target.aigp.simulator import AIGPSimulator


class Controller(BaseController[VelocityNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.program = Program()

    def update(self, telemetry, frames, race_state) -> VelocityNed | None:
        if race_state is None or not race_state.started:
            return None
        return self.program.update(vehicle.state)


if __name__ == "__main__":
    AIGPSimulator(Controller, "vq1.r1").rollout()
