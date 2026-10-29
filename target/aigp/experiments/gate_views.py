"""Stationary training views of gate one through the existing position hold."""

from miniflight import vehicle
from target.aigp.experiments.gate_hold import Controller as GateHold


class Controller(GateHold):
    offsets = ((2, 0, .7), (3.5, -.4, .4), (5, .6, .2), (8, -1, 0), (12, 1.2, -.1), (20, 0, 0))
    max_speed = .005
    hold_seconds = 6

    def __init__(self):
        super().__init__()
        self.offsets = iter(self.offsets)
        self.offset = next(self.offsets)

    def update(self, telemetry, frames, race_state):
        try:
            return super().update(telemetry, frames, race_state)
        except StopIteration:
            self.offset = next(self.offsets)
            self.near_gate = False
            self.started, self.steady = vehicle.state.time, None
            return super().update(telemetry, frames, race_state)
