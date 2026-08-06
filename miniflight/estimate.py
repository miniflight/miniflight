"""Companion-computer attitude estimation in body FRD and world NED."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from common.math import Quaternion, Vector3D


@dataclass(frozen=True)
class MahonyGains:
    kp: float = 0.2
    ki: float = 0.0


class NedMahonyEstimator:
    """Estimate body-to-NED attitude from metric FRD IMU samples."""

    def __init__(
        self,
        initial_yaw_rad: float,
        gains: MahonyGains | None = None,
        required_samples: int = 20,
        acceleration_tolerance_mps2: float = 0.2,
    ) -> None:
        if not math.isfinite(initial_yaw_rad):
            raise ValueError("initial yaw must be finite")
        if required_samples < 1:
            raise ValueError("required samples must be positive")
        if acceleration_tolerance_mps2 <= 0.0:
            raise ValueError("acceleration tolerance must be positive")
        gains = gains or MahonyGains()
        self.initial_yaw_rad = float(initial_yaw_rad)
        self.kp = float(gains.kp)
        self.required_samples = int(required_samples)
        self.acceleration_tolerance_mps2 = float(acceleration_tolerance_mps2)
        self._last_timestamp_ns: Optional[int] = None
        self._initial_accel = []
        self._initial_gyro = []
        self._accel_reference: Optional[tuple[float, float, float]] = None
        self._gyro_bias = Vector3D()
        self._attitude: Optional[Quaternion] = None

    @property
    def initialized(self) -> bool:
        return self._attitude is not None

    @property
    def initialization_progress(self) -> float:
        return min(len(self._initial_accel) / self.required_samples, 1.0)

    @property
    def gyro_bias_rad_s(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._gyro_bias.v)

    @property
    def accel_reference_mps2(self) -> tuple[float, float, float]:
        if self._accel_reference is None:
            raise RuntimeError("acceleration reference is not initialized")
        return self._accel_reference

    @property
    def attitude_body_to_ned(self) -> tuple[float, float, float, float]:
        if self._attitude is None:
            raise RuntimeError("NED attitude is not initialized")
        return tuple(float(value) for value in self._attitude.q)

    def reset(self) -> None:
        self._last_timestamp_ns = None
        self._initial_accel = []
        self._initial_gyro = []
        self._accel_reference = None
        self._gyro_bias = Vector3D()
        self._attitude = None

    def update(
        self,
        timestamp_ns: int,
        accel_frd_mps2: tuple[float, float, float],
        gyro_frd_rad_s: tuple[float, float, float],
    ) -> Optional[tuple[float, float, float, float]]:
        values = (*accel_frd_mps2, *gyro_frd_rad_s)
        if timestamp_ns < 0 or not all(math.isfinite(value) for value in values):
            raise ValueError("IMU sample must be finite")
        if self._last_timestamp_ns is not None and timestamp_ns <= self._last_timestamp_ns:
            return None if self._attitude is None else self.attitude_body_to_ned
        previous_timestamp_ns = self._last_timestamp_ns
        self._last_timestamp_ns = timestamp_ns
        if self._attitude is None:
            self._initial_accel.append(accel_frd_mps2)
            self._initial_gyro.append(gyro_frd_rad_s)
            if len(self._initial_accel) < self.required_samples:
                return None
            count = self.required_samples
            mean = tuple(
                sum(sample[axis] for sample in self._initial_accel) / count
                for axis in range(3)
            )
            self._accel_reference = mean
            gyro_mean = tuple(
                sum(sample[axis] for sample in self._initial_gyro) / count
                for axis in range(3)
            )
            roll = math.atan2(-mean[1], -mean[2])
            pitch = math.atan2(mean[0], math.hypot(mean[1], mean[2]))
            self._attitude = Quaternion.from_euler(roll, pitch, self.initial_yaw_rad)
            self._attitude.normalize()
            self._gyro_bias = Vector3D(*gyro_mean)
            self._initial_accel = []
            self._initial_gyro = []
            return self.attitude_body_to_ned

        dt = (timestamp_ns - previous_timestamp_ns) / 1_000_000_000.0
        corrected_gyro = Vector3D(*gyro_frd_rad_s) - self._gyro_bias
        acceleration_magnitude = math.sqrt(sum(value * value for value in accel_frd_mps2))
        if abs(acceleration_magnitude - 9.80665) <= self.acceleration_tolerance_mps2:
            measured_down = Vector3D(*(
                -value / acceleration_magnitude for value in accel_frd_mps2
            ))
            predicted_down = self._attitude.conjugate().rotate(Vector3D(0.0, 0.0, 1.0))
            corrected_gyro += measured_down.cross(predicted_down) * self.kp
        derivative = self._attitude * Quaternion(0.0, *corrected_gyro.v) * 0.5
        self._attitude = Quaternion(*(self._attitude.q + derivative.q * dt))
        self._attitude.normalize()
        return self.attitude_body_to_ned


__all__ = ["MahonyGains", "NedMahonyEstimator"]
