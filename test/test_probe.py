"""Tests for the reusable read-only MSP machine probe."""

import struct
import unittest

from miniflight.probe import (
    MSP_API_VERSION,
    MSP_BOARD_INFO,
    MSP_FC_VARIANT,
    MSP_FC_VERSION,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_STATUS,
    MspObservationSource,
    MspMachineProbe,
    MspRawImuSource,
)
from miniflight.core.imu import ImuCalibration


class FakeMspClient:
    def __init__(self, responses):
        self.responses = responses
        self.requests = []

    def request(self, command, timeout):
        self.requests.append((command, timeout))
        return self.responses.get(command)


class MspMachineProbeTests(unittest.TestCase):
    def test_probe_reports_only_observed_capabilities(self):
        sensors = (1 << 0) | (1 << 1) | (1 << 5)
        client = FakeMspClient(
            {
                MSP_API_VERSION: b"\x00\x01\x2c",
                MSP_FC_VARIANT: b"BTFL",
                MSP_FC_VERSION: b"\x04\x05\x00",
                MSP_BOARD_INFO: b"MATE\x00",
                MSP_STATUS: struct.pack("<HHHIB", 125, 0, sensors, 0, 0),
                MSP_MOTOR: struct.pack("<8H", 1000, 1001, 1002, 1003, 0, 0, 0, 0),
            }
        )

        profile = MspMachineProbe().inspect(client)

        self.assertEqual(profile.controller, "BTFL")
        self.assertEqual(profile.firmware, "4.5.0")
        self.assertEqual(profile.api_version, "1.44")
        self.assertEqual(profile.board, "MATE")
        self.assertEqual(profile.sensors, frozenset({"acc", "baro", "gyro"}))
        self.assertEqual(profile.motor_count, 4)
        self.assertEqual([command for command, _ in client.requests], [1, 2, 3, 4, 101, 104])

    def test_probe_requires_an_msp_handshake(self):
        with self.assertRaisesRegex(RuntimeError, "No MSP response"):
            MspMachineProbe().inspect(FakeMspClient({MSP_API_VERSION: None}))

    def test_observation_source_requires_explicit_calibration(self):
        client = FakeMspClient(
            {MSP_RAW_IMU: struct.pack("<9h", 1, 2, 3, 4, 5, 6, 0, 0, 0)}
        )
        calibration = ImuCalibration(
            accel_scale_mps2_per_unit=1.0,
            gyro_scale_rad_s_per_unit=1.0,
            accel_bias_raw=(0.0, 0.0, 0.0),
            gyro_bias_raw=(0.0, 0.0, 0.0),
            body_from_sensor=(1, 2, 3),
        )
        clock_values = iter((100, 130))

        tick = MspObservationSource(client, calibration, clock_ns=lambda: next(clock_values)).read()

        self.assertEqual(tick.sequence, 1)
        self.assertEqual(tick.timestamp_ns, 130)
        self.assertEqual(tick.source_age_ns, 30)
        self.assertEqual(tick.accel_mps2, (1.0, 2.0, 3.0))
        self.assertEqual(tick.gyro_rad_s, (4.0, 5.0, 6.0))
        self.assertEqual([command for command, _ in client.requests], [MSP_RAW_IMU])

    def test_raw_source_preserves_controller_units(self):
        client = FakeMspClient(
            {MSP_RAW_IMU: struct.pack("<9h", -1, 2, 3, 4, -5, 6, 0, 0, 0)}
        )
        clock_values = iter((20, 23))

        sample = MspRawImuSource(client, clock_ns=lambda: next(clock_values)).read()

        self.assertEqual(sample.sequence, 1)
        self.assertEqual(sample.accel_raw, (-1.0, 2.0, 3.0))
        self.assertEqual(sample.gyro_raw, (4.0, -5.0, 6.0))
        self.assertEqual(sample.source_age_ns, 3)

    def test_observation_source_rejects_missing_imu_payload(self):
        calibration = ImuCalibration(1.0, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1, 2, 3))

        with self.assertRaisesRegex(RuntimeError, "raw IMU response is unavailable"):
            MspObservationSource(FakeMspClient({MSP_RAW_IMU: None}), calibration).read()


if __name__ == "__main__":
    unittest.main()
