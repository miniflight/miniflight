"""Follow the published R1 gates using the simulator's position controller."""

import math

from common.math import Quaternion, Vector3D
from miniflight import Ned, PositionNed, State
from target.aigp.controllers import BaseController, Gate


def gate_center(gate: Gate) -> Ned:
    """Offset the published gate base to its opening center."""
    norm = math.hypot(*gate.orientation)
    rotation = Quaternion(*(value / norm for value in gate.orientation))
    offset = rotation.rotate(Vector3D(0, 0, -gate.height / 2)).v
    return Ned(*(float(p + d) for p, d in zip(gate.position, offset)))


def gate_target(position, index: int, gates: tuple[Gate, ...]) -> Ned:
    """Choose a point one metre beyond the gate center along the approach."""
    center = gate_center(gates[index])
    direction = tuple(c - p for c, p in zip(center, position))
    distance = math.hypot(*direction)
    if distance < 0.1:
        previous = gate_center(gates[index - 1]) if index else (0.0, 0.0, 0.0)
        direction = tuple(c - p for c, p in zip(center, previous))
        distance = math.hypot(*direction)
    if distance == 0:
        raise ValueError(f"gate {index} has no approach direction")
    return Ned(*(c + d / distance for c, d in zip(center, direction)))


class Controller(BaseController[PositionNed]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.gate = None
        self.target = None

    def update(self, state: State, gate_index: int, gates) -> PositionNed | None:
        motion = state.motion

        if motion is None or not gates:
            return None

        if gate_index < len(gates) and gates[gate_index] != self.gate:
            self.target = PositionNed(*gate_target(motion.position, gate_index, gates))
            self.gate = gates[gate_index]

        return self.target
