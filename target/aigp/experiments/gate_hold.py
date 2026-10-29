"""Hold a fixed NED offset from gate one for a privileged camera measurement."""

import math

from miniflight import PositionNed, vehicle
from target.aigp.controllers import BaseController


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)
    offset = (2, 0, 2*math.tan(math.radians(20)))
    max_speed = .001
    hold_seconds = 3

    def __init__(self):
        self.target = self.started = self.steady = None
        self.near_gate = False

    def update(self, telemetry, frames, race_state):
        state = vehicle.state
        if state is None or state.time is None or state.motion is None or race_state is None or not race_state.started:
            return None
        position, velocity = state.motion.position, state.motion.velocity
        if self.target is None:
            self.target = PositionNed(position.north, position.east, position.down-1)
            self.started = state.time
        if state.time-self.started > 30:
            raise TimeoutError("gate measurement did not settle within 30 device seconds")
        distance, speed = math.dist(position, (self.target.north, self.target.east, self.target.down)), math.hypot(*velocity)
        if not self.near_gate and distance < .05 and speed < .1 and race_state.gate_centers:
            north, east, down = race_state.gate_centers[0]
            dn, de, dd = self.offset
            self.target = PositionNed(north+dn, east+de, down+dd)
            self.near_gate = True
        elif self.near_gate and distance < .02 and speed < self.max_speed and state.gyro is not None and math.hypot(*state.gyro) < .001:
            self.steady = state.time if self.steady is None else self.steady
            if state.time-self.steady >= self.hold_seconds:
                raise StopIteration
        else:
            self.steady = None
        return self.target
