"""Hold a fixed NED offset from gate one for a privileged camera measurement."""

import math
from enum import Enum, auto

from miniflight import PositionNed, vehicle
from target.aigp.controllers import BaseController


class HoldPhase(Enum):
    WAITING_FOR_FEEDBACK = auto()
    STAGING = auto()
    APPROACHING = auto()
    HOLDING = auto()
    COMPLETE = auto()


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)
    offset = (2, 0, 2*math.tan(math.radians(20)))
    max_speed = .001
    hold_seconds = 3

    def __init__(self):
        self.target = self.started = self.steady = None
        self.phase = HoldPhase.WAITING_FOR_FEEDBACK

    def stage_from_current_target(self, stamp):
        self.phase = HoldPhase.STAGING
        self.started, self.steady = stamp, None

    def update(self, telemetry, frames, race_state):
        if self.phase is HoldPhase.COMPLETE:
            raise StopIteration
        state = vehicle.state
        if state is None or state.time is None or state.motion is None or race_state is None or not race_state.started:
            return None
        position, velocity = state.motion.position, state.motion.velocity
        if self.phase is HoldPhase.WAITING_FOR_FEEDBACK:
            self.target = PositionNed(position.north, position.east, position.down-1)
            self.stage_from_current_target(state.time)
        if state.time-self.started > 30:
            raise TimeoutError("gate measurement did not settle within 30 device seconds")
        distance, speed = math.dist(position, (self.target.north, self.target.east, self.target.down)), math.hypot(*velocity)
        if self.phase is HoldPhase.STAGING:
            if distance < .05 and speed < .1 and race_state.gate_centers:
                north, east, down = race_state.gate_centers[0]
                dn, de, dd = self.offset
                self.target = PositionNed(north+dn, east+de, down+dd)
                self.phase = HoldPhase.APPROACHING
        elif distance < .02 and speed < self.max_speed and state.gyro is not None and math.hypot(*state.gyro) < .001:
            if self.phase is HoldPhase.APPROACHING:
                self.steady = state.time
                self.phase = HoldPhase.HOLDING
            if state.time-self.steady >= self.hold_seconds:
                self.phase = HoldPhase.COMPLETE
                raise StopIteration
        else:
            self.steady = None
            self.phase = HoldPhase.APPROACHING
        return self.target
