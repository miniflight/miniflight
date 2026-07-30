"""Passive stationary IMU reference estimation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from miniflight.core.raw_imu import RawImuSample


Vector3 = Tuple[float, float, float]
STANDARD_GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class StationaryImuReference:
    """Measured facts from a caller-admitted stationary raw-IMU window."""

    sample_count: int
    accel_mean_raw: Vector3
    gyro_bias_raw: Vector3
    accel_scale_mps2_per_count: float


class StationaryImuReferenceEstimator:
    """Average one stationary window and freeze its reproducible reference."""

    def __init__(self, required_samples: int) -> None:
        if required_samples <= 0:
            raise ValueError("required_samples must be positive")
        self.required_samples = required_samples
        self._last_sequence: Optional[int] = None
        self._count = 0
        self._accel_sum = [0.0, 0.0, 0.0]
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._reference: Optional[StationaryImuReference] = None

    @property
    def ready(self) -> bool:
        """Return true after the estimator freezes a complete reference."""
        return self._reference is not None

    @property
    def reference(self) -> StationaryImuReference:
        """Return the frozen reference."""
        if self._reference is None:
            raise RuntimeError("stationary IMU reference is not ready")
        return self._reference

    def add(self, sample: RawImuSample) -> bool:
        """Add one admitted stationary raw sample and return completion state."""
        if self._last_sequence is not None and sample.sequence <= self._last_sequence:
            raise RuntimeError("sample sequence must increase")
        self._last_sequence = sample.sequence
        if self.ready:
            return True
        for index in range(3):
            self._accel_sum[index] += sample.accel_raw[index]
            self._gyro_sum[index] += sample.gyro_raw[index]
        self._count += 1
        if self._count == self.required_samples:
            accel_mean = tuple(value / self._count for value in self._accel_sum)
            magnitude = math.sqrt(sum(value * value for value in accel_mean))
            if magnitude <= 0.0:
                raise RuntimeError("stationary acceleration magnitude is zero")
            self._reference = StationaryImuReference(
                sample_count=self._count,
                accel_mean_raw=accel_mean,
                gyro_bias_raw=tuple(value / self._count for value in self._gyro_sum),
                accel_scale_mps2_per_count=STANDARD_GRAVITY_MPS2 / magnitude,
            )
        return self.ready
