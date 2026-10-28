"""Two opposite yaw pulses after the existing stationary gate measurement."""

from miniflight import BodyRates, vehicle
from target.aigp.experiments.gate_hold import Controller as GateHold
from target.aigp.experiments.probe import Probe


class Controller(Probe):
    def __init__(self):
        super().__init__((BodyRates(yaw_rate=.25, thrust=.266), BodyRates(yaw_rate=-.25, thrust=.266)), duration=1)
        self.hold = GateHold()

    def update(self, telemetry, frames, race_state):
        if self.hold is not None:
            try:
                return self.hold.update(telemetry, frames, race_state)
            except StopIteration:
                self.target, self.started_at = self.hold.target, vehicle.state.time
                self.hold = None
        return super().update(telemetry, frames, race_state)
