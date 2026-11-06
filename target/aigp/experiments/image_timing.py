"""Two opposite yaw pulses after the existing stationary gate measurement."""

from enum import Enum, auto

from miniflight import BodyRates, vehicle
from target.aigp.experiments.gate_hold import Controller as GateHold
from target.aigp.experiments.probe import Probe


class TimingPhase(Enum):
    GATE_HOLD = auto()
    PULSES = auto()


class Controller(Probe):
    yaw_rate = .25

    def __init__(self):
        super().__init__((BodyRates(yaw_rate=self.yaw_rate, thrust=.266), BodyRates(yaw_rate=-self.yaw_rate, thrust=.266)), duration=1)
        self.hold = GateHold()
        self.timing_phase = TimingPhase.GATE_HOLD

    def update(self, telemetry, frames, race_state):
        if self.timing_phase is TimingPhase.GATE_HOLD:
            try:
                return self.hold.update(telemetry, frames, race_state)
            except StopIteration:
                self.settle_at(self.hold.target, vehicle.state.time)
                self.timing_phase = TimingPhase.PULSES
        return super().update(telemetry, frames, race_state)
