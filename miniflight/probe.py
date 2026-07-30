"""Read-only MSP machine inspection shared by config and flight programs."""

from __future__ import annotations

import struct
import time
from typing import Optional

from miniflight.core.imu import ImuCalibration
from miniflight.core.machine import MachineProfile
from miniflight.core.observation import ObservationTick
from miniflight.core.raw_imu import RawImuSample


MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104


class MspMachineProbe:
    """Read the identity and available hardware of one MSP flight controller."""

    def inspect(self, client) -> MachineProfile:
        """Return a profile using only read-only MSP request commands."""
        api = client.request(MSP_API_VERSION, timeout=0.5)
        if api is None or len(api) < 3:
            raise RuntimeError("No MSP response")

        variant = client.request(MSP_FC_VARIANT, timeout=0.25)
        version = client.request(MSP_FC_VERSION, timeout=0.25)
        board = client.request(MSP_BOARD_INFO, timeout=0.25)
        status = client.request(MSP_STATUS, timeout=0.25)
        motors = client.request(MSP_MOTOR, timeout=0.25)
        return MachineProfile.from_observation(
            controller=self._text(variant, 4),
            firmware=self._version(version),
            api_version=f"{api[1]}.{api[2]}",
            board=self._text(board, 4),
            sensors=self._sensors(status),
            motor_count=self._motor_count(motors),
        )

    @staticmethod
    def _text(payload: Optional[bytes], length: int) -> Optional[str]:
        if payload is None:
            return None
        value = payload[:length].decode("ascii", errors="replace").strip("\x00 ")
        return value or None

    @staticmethod
    def _version(payload: Optional[bytes]) -> Optional[str]:
        if payload is None or len(payload) < 3:
            return None
        return f"{payload[0]}.{payload[1]}.{payload[2]}"

    @staticmethod
    def _sensors(payload: Optional[bytes]) -> tuple[str, ...]:
        if payload is None or len(payload) < 8:
            return ()
        mask = struct.unpack_from("<I", payload, 4)[0]
        names = ((0, "acc"), (1, "baro"), (2, "mag"), (3, "gps"), (4, "range"), (5, "gyro"))
        return tuple(name for bit, name in names if mask & (1 << bit))

    @staticmethod
    def _motor_count(payload: Optional[bytes]) -> Optional[int]:
        if payload is None or len(payload) < 2:
            return None
        motors = list(struct.unpack_from(f"<{len(payload) // 2}H", payload))
        while motors and motors[-1] == 0:
            motors.pop()
        return len(motors) or None


class MspRawImuSource:
    """Create raw read-only IMU samples from MSP replies.

    This source preserves the controller values. It makes no unit, axis, or
    calibration claim.
    """

    def __init__(self, client, clock_ns=time.monotonic_ns) -> None:
        self._client = client
        self._clock_ns = clock_ns
        self._sequence = 0

    def read(self) -> RawImuSample:
        """Read one raw-IMU frame."""
        started_ns = self._clock_ns()
        payload = self._client.request(MSP_RAW_IMU, timeout=0.05)
        received_ns = self._clock_ns()
        if payload is None or len(payload) < 18:
            raise RuntimeError("MSP raw IMU response is unavailable")
        values = struct.unpack_from("<9h", payload)
        self._sequence += 1
        return RawImuSample(
            sequence=self._sequence,
            timestamp_ns=received_ns,
            accel_raw=tuple(float(value) for value in values[0:3]),
            gyro_raw=tuple(float(value) for value in values[3:6]),
            source_age_ns=max(0, received_ns - started_ns),
        )


class MspObservationSource:
    """Create canonical read-only observation ticks from MSP raw-IMU replies.

    The caller supplies the verified raw scale, bias, and axis mapping. MSP raw
    values alone do not carry enough information to infer those facts safely.
    """

    def __init__(self, client, calibration: ImuCalibration, clock_ns=time.monotonic_ns) -> None:
        self._source = MspRawImuSource(client, clock_ns)
        self._calibration = calibration

    def read(self) -> ObservationTick:
        """Read one raw-IMU frame and return one calibrated observation tick."""
        raw = self._source.read()
        accel_mps2, gyro_rad_s = self._calibration.apply(raw.accel_raw, raw.gyro_raw)
        return ObservationTick(
            sequence=raw.sequence,
            timestamp_ns=raw.timestamp_ns,
            accel_mps2=accel_mps2,
            gyro_rad_s=gyro_rad_s,
            source_age_ns=raw.source_age_ns,
        )
