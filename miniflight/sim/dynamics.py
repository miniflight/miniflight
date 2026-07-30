"""A deterministic rigid-body vacuum model in body FRD and world NED frames."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from miniflight.sim.state import Quaternion, RigidBodyState, Vector3


GRAVITY_MPS2 = 9.80665


def _finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(value) for value in values)


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
    w, x, y, z = quaternion
    q_vector = (x, y, z)
    twice_cross = tuple(2.0 * value for value in _cross(q_vector, vector))
    return tuple(
        vector[index] + w * twice_cross[index] + _cross(q_vector, twice_cross)[index]
        for index in range(3)
    )


def _unit(quaternion: Quaternion) -> Quaternion:
    magnitude = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / magnitude for value in quaternion)


@dataclass(frozen=True)
class RigidBodyModel:
    """The mass and diagonal inertia for one vehicle."""

    mass_kg: float
    inertia_kg_m2: Vector3

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0 or not _finite((self.mass_kg, *self.inertia_kg_m2)):
            raise ValueError("mass and inertia must be finite and positive")
        if any(value <= 0.0 for value in self.inertia_kg_m2):
            raise ValueError("mass and inertia must be finite and positive")


@dataclass(frozen=True)
class BodyWrench:
    """Body-frame total force in N and torque in N m."""

    force_body_n: Vector3
    torque_body_nm: Vector3

    def __post_init__(self) -> None:
        if not _finite((*self.force_body_n, *self.torque_body_nm)):
            raise ValueError("force and torque must be finite")


def step(model: RigidBodyModel, state: RigidBodyState, wrench: BodyWrench, dt_s: float) -> RigidBodyState:
    """Advance one semi-implicit physics step with gravity and rigid-body rotation."""
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    force_world = _rotate(state.attitude_body_to_world, wrench.force_body_n)
    acceleration = tuple(force_world[index] / model.mass_kg for index in range(3))
    acceleration = (acceleration[0], acceleration[1], acceleration[2] + GRAVITY_MPS2)
    position = tuple(
        state.position_world_m[index]
        + state.velocity_world_mps[index] * dt_s
        + 0.5 * acceleration[index] * dt_s * dt_s
        for index in range(3)
    )
    velocity = tuple(
        state.velocity_world_mps[index] + acceleration[index] * dt_s for index in range(3)
    )
    inertia_rate = tuple(
        model.inertia_kg_m2[index] * state.body_rates_rad_s[index] for index in range(3)
    )
    coupling = _cross(state.body_rates_rad_s, inertia_rate)
    rates = tuple(
        state.body_rates_rad_s[index]
        + (wrench.torque_body_nm[index] - coupling[index]) / model.inertia_kg_m2[index] * dt_s
        for index in range(3)
    )
    w, x, y, z = state.attitude_body_to_world
    p, q, r = rates
    attitude = _unit((
        w + 0.5 * (-x * p - y * q - z * r) * dt_s,
        x + 0.5 * (w * p + y * r - z * q) * dt_s,
        y + 0.5 * (w * q - x * r + z * p) * dt_s,
        z + 0.5 * (w * r + x * q - y * p) * dt_s,
    ))
    return RigidBodyState(
        timestamp_ns=state.timestamp_ns + round(dt_s * 1_000_000_000),
        position_world_m=position,
        velocity_world_mps=velocity,
        attitude_body_to_world=attitude,
        body_rates_rad_s=rates,
    )
