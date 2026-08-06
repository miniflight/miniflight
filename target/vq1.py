"""Anduril VQ1 control plane."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable

from miniflight.program import (
    AccelerationNed,
    Attitude,
    BodyRates,
    PositionNed,
    VelocityNed,
)


@dataclass(frozen=True)
class Vq1CollectiveModel:
    """Convert physical collective acceleration to the VQ1 wire value."""

    hover_accel_mps2: float = 9.80665
    hover_wire_value: float = 0.264
    reference_accel_mps2: float = 19.7
    reference_wire_value: float = 0.450
    minimum_wire_value: float = 0.15
    maximum_wire_value: float = 0.45

    def wire_value(self, acceleration_mps2: float) -> float:
        slope = (
            (self.reference_wire_value - self.hover_wire_value)
            / (self.reference_accel_mps2 - self.hover_accel_mps2)
        )
        value = self.hover_wire_value + (
            acceleration_mps2 - self.hover_accel_mps2
        ) * slope
        return max(self.minimum_wire_value, min(self.maximum_wire_value, value))


class Vq1Link:
    """Send explicit commands through the VQ1 MAVLink transport."""

    def __init__(
        self,
        link,
        boot_ms: int,
        mavlink,
        collective: Vq1CollectiveModel | None = None,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self._link = link
        self._boot_ms = int(boot_ms)
        self._mavlink = mavlink
        self._collective = collective or Vq1CollectiveModel()
        self._clock_ms = clock_ms or (lambda: int(time.time() * 1000))

    def send_position_ned(self, command: PositionNed) -> None:
        mask = (
            self._mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._submit_local_ned(
            mask,
            command.position_ned_m,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            command.yaw_rad,
        )

    def send_velocity_ned(self, command: VelocityNed) -> None:
        mask = (
            self._mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._submit_local_ned(
            mask,
            (0.0, 0.0, 0.0),
            command.velocity_ned_mps,
            (0.0, 0.0, 0.0),
            command.yaw_rad,
        )

    def send_acceleration_ned(self, command: AccelerationNed) -> None:
        mask = (
            self._mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | self._mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._submit_local_ned(
            mask,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            command.acceleration_ned_mps2,
            command.yaw_rad,
        )

    def _submit_local_ned(self, mask, position, velocity, acceleration, yaw_rad) -> None:
        yaw = 0.0 if yaw_rad is None else yaw_rad
        if yaw_rad is None:
            mask |= self._mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        self._link.mav.set_position_target_local_ned_send(
            self._clock_ms() - self._boot_ms,
            self._link.target_system,
            self._link.target_component,
            self._mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            *position,
            *velocity,
            *acceleration,
            yaw,
            0.0,
        )

    def send_attitude(self, command: Attitude) -> None:
        mask = (
            self._mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | self._mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | self._mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            | 16
        )
        self._link.mav.set_attitude_target_send(
            self._clock_ms() - self._boot_ms,
            self._link.target_system,
            self._link.target_component,
            mask,
            list(command.attitude_body_to_ned),
            0.0,
            0.0,
            0.0,
            self._collective.wire_value(command.collective_accel_mps2),
        )

    def send_body_rates(self, command: BodyRates) -> None:
        mask = self._mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE | 16
        self._link.mav.set_attitude_target_send(
            self._clock_ms() - self._boot_ms,
            self._link.target_system,
            self._link.target_component,
            mask,
            [1.0, 0.0, 0.0, 0.0],
            command.roll_rate_rad_s,
            command.pitch_rate_rad_s,
            command.yaw_rate_rad_s,
            self._collective.wire_value(command.collective_accel_mps2),
        )


__all__ = ["Vq1CollectiveModel", "Vq1Link"]
