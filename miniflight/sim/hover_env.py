"""A small deterministic hover environment at the Miniflight command boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from miniflight.program import BodyCommand
from miniflight.sim.dynamics import BodyWrench, RigidBodyModel, step
from miniflight.sim.state import RigidBodyState, Vector3


@dataclass(frozen=True)
class HoverEnvironmentModel:
    """Plant parameters for rate-and-collective policy tests."""

    body: RigidBodyModel
    rate_time_constant_s: float
    max_angular_accel_rad_s2: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.rate_time_constant_s) or self.rate_time_constant_s <= 0.0:
            raise ValueError("rate_time_constant_s must be finite and positive")
        if not math.isfinite(self.max_angular_accel_rad_s2) or self.max_angular_accel_rad_s2 <= 0.0:
            raise ValueError("max_angular_accel_rad_s2 must be finite and positive")


class HoverEnvironment:
    """Deterministic full-state environment for policies at the body-command boundary.

    State is a perfect simulator observation. It is useful for initial policy
    behavior tests, but is not a sensor-transfer claim.
    """

    def __init__(self, model: HoverEnvironmentModel, dt_s: float, horizon_steps: int) -> None:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        if horizon_steps <= 0:
            raise ValueError("horizon_steps must be positive")
        self.model = model
        self.dt_s = dt_s
        self.horizon_steps = horizon_steps
        self.steps = 0
        self.state = self.reset()

    def reset(self, state: Optional[RigidBodyState] = None) -> RigidBodyState:
        """Reset to an explicit state or level stationary hover origin."""
        self.steps = 0
        self.state = state or RigidBodyState(
            timestamp_ns=0,
            position_world_m=(0.0, 0.0, 0.0),
            velocity_world_mps=(0.0, 0.0, 0.0),
            attitude_body_to_world=(1.0, 0.0, 0.0, 0.0),
            body_rates_rad_s=(0.0, 0.0, 0.0),
        )
        return self.state

    def step(self, command: BodyCommand) -> tuple[RigidBodyState, bool]:
        """Advance one policy command and return state plus time-limit terminal flag."""
        rates = self.state.body_rates_rad_s
        requested_rates = (command.roll_rate_rad_s, command.pitch_rate_rad_s, command.yaw_rate_rad_s)
        requested_accel = tuple(
            max(
                -self.model.max_angular_accel_rad_s2,
                min(
                    self.model.max_angular_accel_rad_s2,
                    (requested_rates[index] - rates[index]) / self.model.rate_time_constant_s,
                ),
            )
            for index in range(3)
        )
        torque = tuple(
            self.model.body.inertia_kg_m2[index] * requested_accel[index]
            for index in range(3)
        )
        wrench = BodyWrench(
            force_body_n=(0.0, 0.0, -self.model.body.mass_kg * command.collective_accel_mps2),
            torque_body_nm=torque,
        )
        self.state = step(self.model.body, self.state, wrench, self.dt_s)
        self.steps += 1
        return self.state, self.steps >= self.horizon_steps
