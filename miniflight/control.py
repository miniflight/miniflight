from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class Control:
    """Body rates in rad/s and collective thrust in [0, 1]."""

    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    thrust: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (
            self.roll_rate, self.pitch_rate, self.yaw_rate, self.thrust
        )):
            raise ValueError("body rates and thrust must be finite")
        if not 0.0 <= self.thrust <= 1.0:
            raise ValueError("thrust must be between 0 and 1")


@dataclass(frozen=True)
class PositionNed:
    """Absolute position in metres; the simulator owns the position-control loop."""

    north: float
    east: float
    down: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.north, self.east, self.down)):
            raise ValueError("NED setpoints must be finite")


@dataclass(frozen=True)
class Frame:
    id: int
    time_ns: int
    received_at: float  # host monotonic seconds
    bgr: np.ndarray


@dataclass(frozen=True)
class Race:
    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int  # unchanged wire value
    received_at: float = 0.0  # host monotonic seconds


@dataclass(frozen=True)
class State:
    time: float  # simulator IMU timestamp, seconds
    dt: float  # simulator time since the previous update; zero on the first
    acceleration: tuple[float, float, float]  # body x/y/z, m/s²
    gyro: tuple[float, float, float]  # body x/y/z, rad/s
    frame: Frame | None
    race: Race | None
    telemetry: Mapping[str, Any]  # latest unmodified MAVLink messages by type
    received_at: Mapping[str, float]  # host monotonic receipt times by type
    messages: tuple[Any, ...]  # packets received since the previous read
