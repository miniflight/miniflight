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
        self.gate_started_at = None

    def update(self, state: State, gate_index: int, gates) -> PositionNed | None:
        motion = state.motion
        if motion is None or not gates:
            if self.gate is not None:
                raise ValueError("r1_gates needs fresh VQ1 position and track geometry")
            return None  # Wait for startup telemetry without arming.

        position = motion.position
        if not all(math.isfinite(v) for v in position):
            raise ValueError("R1 position telemetry must be finite")

        index = gate_index
        if not 0 <= index <= len(gates):
            raise ValueError(f"gate index {index} does not belong to the published track")
        if self.gate is not None and index < self.gate:
            raise ValueError("race reset during control; start a new run")
        if index == len(gates):
            return self.target  # Hold the final target until the native finish signal.

        if index != self.gate:
            # Hold this target until the simulator reports the gate was passed.
            self.target = gate_target(position, index, gates)
            self.gate, self.gate_started_at = index, state.time
        if state.time - self.gate_started_at > 45:
            raise TimeoutError(f"gate {index + 1} was not passed within 45 simulator seconds")
        return self.target
