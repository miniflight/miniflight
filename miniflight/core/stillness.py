"""Consecutive quiet-sample gate for IMU calibration."""

from __future__ import annotations

import math
from typing import Optional

from miniflight.core.observation import ObservationTick


class StillnessDetector:
    """Require a bounded consecutive window of low-motion IMU samples."""

    STANDARD_GRAVITY_MPS2 = 9.80665

    def __init__(
        self,
        required_samples: int,
        max_gyro_rad_s: float,
        max_accel_error_mps2: float,
    ) -> None:
        if required_samples <= 0:
            raise ValueError("required_samples must be positive")
        if max_gyro_rad_s < 0.0 or max_accel_error_mps2 < 0.0:
            raise ValueError("stillness limits must be non-negative")
        self.required_samples = required_samples
        self.max_gyro_rad_s = max_gyro_rad_s
        self.max_accel_error_mps2 = max_accel_error_mps2
        self.accepted_samples = 0
        self._last_sequence: Optional[int] = None

    @property
    def is_still(self) -> bool:
        """Return true only after the required consecutive quiet samples."""
        return self.accepted_samples >= self.required_samples

    def add(self, tick: ObservationTick) -> bool:
        """Add one tick and return whether the full quiet window is complete."""
        if self._last_sequence is not None and tick.sequence <= self._last_sequence:
            raise RuntimeError("tick sequence must increase")
        self._last_sequence = tick.sequence

        gyro_norm = math.sqrt(sum(value * value for value in tick.gyro_rad_s))
        accel_norm = math.sqrt(sum(value * value for value in tick.accel_mps2))
        quiet = (
            gyro_norm <= self.max_gyro_rad_s
            and abs(accel_norm - self.STANDARD_GRAVITY_MPS2) <= self.max_accel_error_mps2
        )
        self.accepted_samples = self.accepted_samples + 1 if quiet else 0
        return self.is_still

    def reset(self) -> None:
        """Discard the current calibration window."""
        self.accepted_samples = 0
        self._last_sequence = None
