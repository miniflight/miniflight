"""One immutable sensor snapshot for one Miniflight lockstep."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ObservationTick:
    """All program input that is valid at one monotonic timestamp.

    The body frame is forward, right, down. Acceleration is specific force in
    m/s². Angular velocity is rad/s. Camera bytes are opaque to the runtime.
    A missing camera is valid during IMU-only qualification and calibration.
    """

    sequence: int
    timestamp_ns: int
    accel_mps2: Vector3
    gyro_rad_s: Vector3
    source_age_ns: int
    camera_data: Optional[bytes] = None
    camera_timestamp_ns: Optional[int] = None
    camera_width: Optional[int] = None
    camera_height: Optional[int] = None
    camera_encoding: Optional[str] = None

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        if self.source_age_ns < 0:
            raise ValueError("source_age_ns must be non-negative")
        if len(self.accel_mps2) != 3 or len(self.gyro_rad_s) != 3:
            raise ValueError("IMU vectors must contain exactly three values")
        if not all(math.isfinite(value) for value in (*self.accel_mps2, *self.gyro_rad_s)):
            raise ValueError("IMU values must be finite")

        has_camera = self.camera_data is not None
        camera_fields = (
            self.camera_timestamp_ns,
            self.camera_width,
            self.camera_height,
            self.camera_encoding,
        )
        if has_camera != all(field is not None for field in camera_fields):
            raise ValueError("camera data requires timestamp, size, and encoding")
        if not has_camera:
            return
        if self.camera_timestamp_ns > self.timestamp_ns:
            raise ValueError("camera timestamp cannot be newer than the tick")
        if self.camera_width <= 0 or self.camera_height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not self.camera_encoding:
            raise ValueError("camera encoding is required")

    @property
    def has_camera(self) -> bool:
        """Return true only when a complete camera sample is present."""
        return self.camera_data is not None
