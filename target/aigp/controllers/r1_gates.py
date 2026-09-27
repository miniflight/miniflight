"""Six-gate R1 baseline using the simulator's built-in position controller."""

import math

from miniflight import PositionNed, State
from target.aigp.controllers import BaseController


# R1 NED gate centers, retained from examples/aigp/thread_gates.py.
GATES = (
    (-23.297967744257804, -0.3999021772411884, -1.3919580206274782),
    (-46.89374907055175, -2.499990058329445, 3.7080417871475424),
    (-74.5937498334912, 1.200009870144981, 12.308041214942953),
    (-111.49374372997558, -5.099989724543434, 23.208040833473227),
    (-135.4937437299756, -0.7999900702503742, 23.99565374851229),
    (-159.19374067821778, -4.399989915278297, 24.6080404520035),
)


def gate_target(position, index: int) -> PositionNed:
    """Aim one metre beyond the gate center along the approach from position."""
    center = GATES[index]
    direction = tuple(c - p for c, p in zip(center, position))
    distance = math.hypot(*direction)
    if distance < 0.1:
        previous = GATES[index - 1] if index else (0.0, 0.0, 0.0)
        direction = tuple(c - p for c, p in zip(center, previous))
        distance = math.hypot(*direction)
    return PositionNed(*(c + d / distance for c, d in zip(center, direction)))


class Controller(BaseController):
    targets = ("vq1.r1",)

    def __init__(self):
        self.gate = None
        self.target = None
        self.gate_started_at = None

    def update(self, state: State, gate_index: int) -> PositionNed | None:
        motion = state.motion
        if motion is None:
            if self.gate is not None:
                raise ValueError("r1_gates needs fresh VQ1 position telemetry")
            return None  # Wait for startup telemetry without arming.

        position = motion.position
        if not all(math.isfinite(v) for v in position):
            raise ValueError("R1 position telemetry must be finite")

        index = gate_index
        if not 0 <= index <= len(GATES):
            raise ValueError(f"gate index {index} does not belong to the six-gate R1 course")
        if self.gate is not None and index < self.gate:
            raise ValueError("race reset during control; start a new run")
        if index == len(GATES):
            return self.target  # Hold the final target until the native finish signal.

        if index != self.gate:
            # Hold this target until the simulator reports the gate was passed.
            self.target = gate_target(position, index)
            self.gate, self.gate_started_at = index, state.time
        if state.time - self.gate_started_at > 45:
            raise TimeoutError(f"gate {index + 1} was not passed within 45 simulator seconds")
        return self.target
