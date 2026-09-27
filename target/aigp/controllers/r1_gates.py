"""Follow the published R1 gates using the simulator's position controller."""

import math

from miniflight import PositionNed, State
from target.aigp.controllers import BaseController


def gate_target(position, index: int, gates) -> PositionNed:
    """Aim one metre beyond the gate center along the approach from position."""
    center = gates[index].center
    direction = tuple(c - p for c, p in zip(center, position))
    distance = math.hypot(*direction)
    if distance < 0.1:
        previous = gates[index - 1].center if index else (0.0, 0.0, 0.0)
        direction = tuple(c - p for c, p in zip(center, previous))
        distance = math.hypot(*direction)
    return PositionNed(*(c + d / distance for c, d in zip(center, direction)))


class Controller(BaseController):
    targets = ("vq1.r1",)

    def __init__(self):
        self.gate = None
        self.target = None

    def update(self, state: State, gate_index: int, gates) -> PositionNed | None:
        time = state.time
        dt = state.dt
        acceleration = state.acceleration
        gyro = state.gyro
        received_at = state.received_at
        frame = state.frame
        motion = state.motion
        attitude = state.attitude
        motors = state.motors

        if motion is None or not gates:
            return None

        if gate_index < len(gates) and gate_index != self.gate:
            self.target = gate_target(motion.position, gate_index, gates)
            self.gate = gate_index

        return self.target
