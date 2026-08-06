"""Flight programs and their hardware-independent commands."""

from __future__ import annotations

from abc import ABC, abstractmethod
import math
from dataclasses import dataclass
from typing import Optional, Tuple, Union


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _finite(values: tuple[float, ...], name: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} values must be finite")


@dataclass(frozen=True)
class PositionNed:
    """Position target in meters in the world NED frame."""

    position_ned_m: Vector3
    yaw_rad: Optional[float] = None

    def __post_init__(self) -> None:
        if len(self.position_ned_m) != 3:
            raise ValueError("position_ned_m must contain three values")
        values = self.position_ned_m
        if self.yaw_rad is not None:
            values = (*values, self.yaw_rad)
        _finite(tuple(values), "position command")


@dataclass(frozen=True)
class VelocityNed:
    """Velocity target in meters per second in the world NED frame."""

    velocity_ned_mps: Vector3
    yaw_rad: Optional[float] = None

    def __post_init__(self) -> None:
        if len(self.velocity_ned_mps) != 3:
            raise ValueError("velocity_ned_mps must contain three values")
        values = self.velocity_ned_mps
        if self.yaw_rad is not None:
            values = (*values, self.yaw_rad)
        _finite(tuple(values), "velocity command")


@dataclass(frozen=True)
class AccelerationNed:
    """Acceleration target in meters per second squared in world NED."""

    acceleration_ned_mps2: Vector3
    yaw_rad: Optional[float] = None

    def __post_init__(self) -> None:
        if len(self.acceleration_ned_mps2) != 3:
            raise ValueError("acceleration_ned_mps2 must contain three values")
        values = self.acceleration_ned_mps2
        if self.yaw_rad is not None:
            values = (*values, self.yaw_rad)
        _finite(tuple(values), "acceleration command")


@dataclass(frozen=True)
class Attitude:
    """Body-to-NED attitude target and collective acceleration."""

    attitude_body_to_ned: Quaternion
    collective_accel_mps2: float

    def __post_init__(self) -> None:
        if len(self.attitude_body_to_ned) != 4:
            raise ValueError("attitude_body_to_ned must contain four values")
        _finite((*self.attitude_body_to_ned, self.collective_accel_mps2), "attitude command")
        norm = math.sqrt(sum(value * value for value in self.attitude_body_to_ned))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("attitude_body_to_ned must have unit length")
        if self.collective_accel_mps2 < 0.0:
            raise ValueError("collective_accel_mps2 must be non-negative")


@dataclass(frozen=True)
class BodyRates:
    """Body FRD rate target and collective acceleration."""

    collective_accel_mps2: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        values = (
            self.collective_accel_mps2,
            self.roll_rate_rad_s,
            self.pitch_rate_rad_s,
            self.yaw_rate_rad_s,
        )
        _finite(values, "body-rate command")
        if self.collective_accel_mps2 < 0.0:
            raise ValueError("collective_accel_mps2 must be non-negative")


@dataclass(frozen=True)
class MotorThrusts:
    """Per-motor thrust targets in newtons."""

    thrust_n: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.thrust_n:
            raise ValueError("motor thrust command must contain a motor")
        _finite(self.thrust_n, "motor thrust command")
        if any(value < 0.0 for value in self.thrust_n):
            raise ValueError("motor thrust values must be non-negative")


FlightCommand = Union[
    PositionNed,
    VelocityNed,
    AccelerationNed,
    Attitude,
    BodyRates,
    MotorThrusts,
]


class FlightProgram(ABC):
    """One flight behavior with no target or board access."""

    name: str

    def start(self) -> None:
        """Initialize program-local state."""

    @abstractmethod
    def step(self) -> FlightCommand:
        """Return the next command."""

    def stop(self, reason: str) -> None:
        """Release program-local state."""


__all__ = [
    "AccelerationNed",
    "Attitude",
    "BodyRates",
    "FlightCommand",
    "FlightProgram",
    "MotorThrusts",
    "PositionNed",
    "VelocityNed",
]
