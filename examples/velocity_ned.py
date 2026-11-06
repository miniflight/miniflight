"""Climb one metre, move north for two seconds, then observe speed below 0.1 m/s."""

import math
from enum import Enum, auto

from miniflight import State, VelocityNed


class ManeuverPhase(Enum):
    WAITING_FOR_FEEDBACK = auto()
    CLIMBING = auto()
    MOVING_NORTH = auto()
    BRAKING = auto()
    COMPLETE = auto()


class Program:
    def __init__(self):
        self.started_at = self.moving_at = self.braking_time = self.lift_down = None
        self.phase = ManeuverPhase.WAITING_FOR_FEEDBACK

    def update(self, state: State | None) -> VelocityNed | None:
        if self.phase is ManeuverPhase.COMPLETE:
            raise StopIteration
        feedback = None if state is None else state.rate_feedback
        if feedback is None or state.motion is None:
            return None
        if self.phase is ManeuverPhase.WAITING_FOR_FEEDBACK:
            self.started_at = feedback.time
            self.lift_down = state.motion.position.down - 1
            self.phase = ManeuverPhase.CLIMBING
        elapsed = feedback.time - self.started_at
        if elapsed >= 8:
            raise TimeoutError("velocity maneuver did not complete within eight device seconds")
        if self.phase is ManeuverPhase.CLIMBING:
            if state.motion.position.down > self.lift_down:
                return VelocityNed(0, 0, -1)
            self.moving_at = feedback.time
            self.phase = ManeuverPhase.MOVING_NORTH
        if self.phase is ManeuverPhase.MOVING_NORTH:
            if feedback.time - self.moving_at < 2:
                return VelocityNed(-1, 0, 0)
            self.braking_time = state.motion.time
            self.phase = ManeuverPhase.BRAKING
        elif state.motion.time > self.braking_time and math.hypot(*state.motion.velocity) < .1:
            self.phase = ManeuverPhase.COMPLETE
            raise StopIteration
        return VelocityNed(0, 0, 0)
