"""Explicit conversion from one sensor frame into Miniflight IMU units."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ImuCalibration:
    """Convert raw accelerometer and gyro values into body-frame SI units.

    `body_from_sensor` is a signed one-based axis permutation. For example,
    `(2, -1, 3)` means body forward receives sensor axis 2, body right receives
    negative sensor axis 1, and body down receives sensor axis 3.
    """

    accel_scale_mps2_per_unit: float
    gyro_scale_rad_s_per_unit: float
    accel_bias_raw: Vector3
    gyro_bias_raw: Vector3
    body_from_sensor: Tuple[int, int, int]

    def __post_init__(self) -> None:
        if self.accel_scale_mps2_per_unit <= 0.0 or self.gyro_scale_rad_s_per_unit <= 0.0:
            raise ValueError("IMU scales must be positive")
        values = (*self.accel_bias_raw, *self.gyro_bias_raw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("IMU bias values must be finite")
        axes = tuple(abs(axis) for axis in self.body_from_sensor)
        if len(self.body_from_sensor) != 3 or set(axes) != {1, 2, 3}:
            raise ValueError("body_from_sensor must be a signed axis permutation")

    def apply(self, accel_raw: Vector3, gyro_raw: Vector3) -> tuple[Vector3, Vector3]:
        """Return body-frame acceleration in m/s² and gyro rate in rad/s."""
        if len(accel_raw) != 3 or len(gyro_raw) != 3:
            raise ValueError("raw IMU vectors must contain exactly three values")
        values = (*accel_raw, *gyro_raw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("raw IMU values must be finite")
        accel_sensor = tuple(
            (raw - bias) * self.accel_scale_mps2_per_unit
            for raw, bias in zip(accel_raw, self.accel_bias_raw)
        )
        gyro_sensor = tuple(
            (raw - bias) * self.gyro_scale_rad_s_per_unit
            for raw, bias in zip(gyro_raw, self.gyro_bias_raw)
        )
        return self._body_vector(accel_sensor), self._body_vector(gyro_sensor)

    def _body_vector(self, sensor_vector: Vector3) -> Vector3:
        return tuple(
            sensor_vector[axis - 1] if axis > 0 else -sensor_vector[-axis - 1]
            for axis in self.body_from_sensor
        )
