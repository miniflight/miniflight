"""The minimum six-degree-of-freedom vehicle state."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class RigidBodyState:
    """World position and velocity with body-to-world attitude and body rates."""

    timestamp_ns: int
    position_world_m: Vector3
    velocity_world_mps: Vector3
    attitude_body_to_world: Quaternion
    body_rates_rad_s: Vector3

    def __post_init__(self) -> None:
        values = (
            *self.position_world_m,
            *self.velocity_world_mps,
            *self.attitude_body_to_world,
            *self.body_rates_rad_s,
        )
        if self.timestamp_ns < 0 or not all(math.isfinite(value) for value in values):
            raise ValueError("state must have a non-negative timestamp and finite values")
        norm = math.sqrt(sum(value * value for value in self.attitude_body_to_world))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("attitude quaternion must have unit length")
