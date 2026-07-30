"""Stationary gyro-bias estimation for one calibration cycle."""

from __future__ import annotations

import math
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]


class GyroBiasEstimator:
    """Average an admitted stationary raw-gyro window, then freeze the result."""

    def __init__(self, required_samples: int) -> None:
        if required_samples <= 0:
            raise ValueError("required_samples must be positive")
        self.required_samples = required_samples
        self._count = 0
        self._sum = [0.0, 0.0, 0.0]
        self._last_sequence: Optional[int] = None
        self._bias_raw: Optional[Vector3] = None

    @property
    def sample_count(self) -> int:
        """Return the number of samples included in the frozen estimate."""
        return self._count

    @property
    def ready(self) -> bool:
        """Return true only when the estimator has frozen its bias."""
        return self._bias_raw is not None

    @property
    def bias_raw(self) -> Vector3:
        """Return the frozen raw gyro bias after calibration completes."""
        if self._bias_raw is None:
            raise RuntimeError("gyro bias is not ready")
        return self._bias_raw

    def add(self, sequence: int, gyro_raw: Vector3) -> bool:
        """Add one stationary raw-gyro sample and return completion state."""
        if sequence < 0:
            raise ValueError("sequence must be non-negative")
        if self._last_sequence is not None and sequence <= self._last_sequence:
            raise RuntimeError("sample sequence must increase")
        if len(gyro_raw) != 3 or not all(math.isfinite(value) for value in gyro_raw):
            raise ValueError("raw gyro must contain three finite values")
        self._last_sequence = sequence
        if self.ready:
            return True

        for index, value in enumerate(gyro_raw):
            self._sum[index] += value
        self._count += 1
        if self._count == self.required_samples:
            self._bias_raw = tuple(value / self._count for value in self._sum)
        return self.ready

    def reset(self) -> None:
        """Discard the partial or frozen estimate."""
        self._count = 0
        self._sum = [0.0, 0.0, 0.0]
        self._last_sequence = None
        self._bias_raw = None
