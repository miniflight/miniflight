"""Timestamped observations shared by controllers and target adapters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple, TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np


class Ned(NamedTuple):
    """North, east, down components; units belong to the containing field."""

    north: float
    east: float
    down: float


class RateSample(NamedTuple):
    time: float  # device seconds
    received_at: float  # host monotonic seconds
    gyro: tuple[float, float, float]  # FRD rad/s


@dataclass(frozen=True)
class Motion:
    time: float  # device seconds
    received_at: float  # host monotonic seconds
    position: Ned  # metres in the target's local NED frame
    velocity: Ned  # m/s in the same frame


@dataclass(frozen=True)
class Attitude:
    time: float
    received_at: float
    roll: float  # radians, FRD body relative to local NED
    pitch: float
    yaw: float
    angular_velocity: tuple[float, float, float] | None = None  # FRD rad/s on this sample's clock.


@dataclass(frozen=True)
class MotorOutputs:
    """Reported output channels in target-defined units, not measured RPM."""

    time: float
    received_at: float
    outputs: tuple[float, ...]
    active: int  # channel bitmask


@dataclass(frozen=True)
class Frame:
    id: int
    time_ns: int  # device timestamp
    received_at: float
    bgr: np.ndarray  # pixel storage is supplied by the camera adapter


@dataclass(frozen=True)
class State:
    """Latest accepted observations, each with its own sample clock.

    Device time and host receipt time are separate clocks. Each optional sample
    keeps its own timestamp; dt belongs to the returned IMU observations, not to
    every sensor or control stage. Controller memory is not part of this record.
    """

    time: float | None = None  # device IMU timestamp in seconds
    dt: float = 0.0  # IMU seconds since the previous read; first read is zero
    acceleration: tuple[float, float, float] | None = None  # reported FRD body acceleration, m/s²
    gyro: tuple[float, float, float] | None = None  # FRD body angular velocity, rad/s
    received_at: float | None = None  # IMU receipt time, host monotonic seconds
    frame: Frame | None = None
    motion: Motion | None = None
    attitude: Attitude | None = None
    motors: MotorOutputs | None = None
    rate_samples: tuple[RateSample, ...] = ()  # New accepted gyros in this read, in arrival order.

    @property
    def rate_feedback(self) -> RateSample | None:
        """Measured IMU angular velocity, retaining its sample and receipt clocks."""
        return None if self.time is None or self.received_at is None or self.gyro is None else RateSample(self.time, self.received_at, self.gyro)
