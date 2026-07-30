"""Observed machine capabilities shared by config and flight programs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Iterable, Optional


@dataclass(frozen=True)
class MachineProfile:
    """Facts observed from one connected flight controller.

    This is a report, not a configuration. Programs use it to select supported
    behavior. For example, hover can use a barometer only when it is present.
    """

    controller: Optional[str]
    firmware: Optional[str]
    api_version: Optional[str]
    board: Optional[str]
    sensors: FrozenSet[str]
    motor_count: Optional[int] = None

    @classmethod
    def from_observation(
        cls,
        *,
        controller: Optional[str],
        firmware: Optional[str],
        board: Optional[str],
        sensors: Iterable[str],
        motor_count: Optional[int] = None,
        api_version: Optional[str] = None,
    ) -> "MachineProfile":
        """Normalize one probe result into stable, lowercase capability names."""
        normalized_sensors = frozenset(
            sensor.strip().lower() for sensor in sensors if sensor and sensor.strip()
        )
        if motor_count is not None and motor_count <= 0:
            raise ValueError("motor_count must be positive when known")
        return cls(
            controller=controller,
            firmware=firmware,
            api_version=api_version,
            board=board,
            sensors=normalized_sensors,
            motor_count=motor_count,
        )

    @property
    def has_imu(self) -> bool:
        """Return true only when both required IMU sensors are present."""
        return self.supports("acc", "gyro")

    def supports(self, *required_sensors: str) -> bool:
        """Return true only when every requested optional sensor is present."""
        required = {sensor.strip().lower() for sensor in required_sensors}
        return required.issubset(self.sensors)
