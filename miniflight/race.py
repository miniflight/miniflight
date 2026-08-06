"""Metric race geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math

from miniflight.core.observation import Vector3
from miniflight.program import FlightObservation, FlightProgram, PositionNed


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


@dataclass(frozen=True)
class Gate:
    """An upright opening with an explicit course direction in world NED."""

    center_ned_m: Vector3
    course_normal_ned: Vector3
    width_m: float
    height_m: float

    def __post_init__(self) -> None:
        values = (
            *self.center_ned_m,
            *self.course_normal_ned,
            self.width_m,
            self.height_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("gate values must be finite")
        if self.width_m <= 0.0 or self.height_m <= 0.0:
            raise ValueError("gate dimensions must be positive")
        magnitude = _norm(self.course_normal_ned)
        if magnitude <= 1e-6:
            raise ValueError("gate course normal must be nonzero")
        normal = tuple(value / magnitude for value in self.course_normal_ned)
        if math.hypot(normal[0], normal[1]) <= 1e-6:
            raise ValueError("gate course normal must have a horizontal component")
        object.__setattr__(self, "course_normal_ned", normal)


class ThreadGatesProgram(FlightProgram):
    """Send the center of the active gate as a position NED command."""

    name = "thread-gates"

    def __init__(self, gate_centers_ned_m: tuple[Vector3, ...]) -> None:
        if not gate_centers_ned_m:
            raise ValueError("thread-gates program requires a gate")
        self._commands = tuple(PositionNed(center) for center in gate_centers_ned_m)
        self._active_gate = 0

    def select_gate(self, index: int) -> None:
        if index < 0 or index >= len(self._commands):
            raise IndexError("active gate is outside the course")
        self._active_gate = index

    def step(self, observation: FlightObservation) -> PositionNed:
        return self._commands[self._active_gate]


__all__ = ["Gate", "ThreadGatesProgram"]
