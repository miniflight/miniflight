"""Unscaled IMU data from a hardware transport."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class RawImuSample:
    """One timestamped raw accelerometer and gyroscope sample.

    The values have no declared unit or body-frame meaning. Calibration must
    establish those facts before a flight program receives this sensor data.
    """

    sequence: int
    timestamp_ns: int
    accel_raw: Vector3
    gyro_raw: Vector3
    source_age_ns: int

    def __post_init__(self) -> None:
        if self.sequence < 0 or self.timestamp_ns < 0 or self.source_age_ns < 0:
            raise ValueError("raw IMU sequence and timestamps must be non-negative")
        if not all(math.isfinite(value) for value in (*self.accel_raw, *self.gyro_raw)):
            raise ValueError("raw IMU vectors must contain finite values")
