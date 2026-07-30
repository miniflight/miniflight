"""Tests for machine capabilities used by config and flight programs."""

import unittest

from miniflight.core.machine import MachineProfile


class MachineProfileTests(unittest.TestCase):
    def test_profile_normalizes_sensor_capabilities(self):
        profile = MachineProfile.from_observation(
            controller="BTFL",
            firmware="4.5.0",
            api_version="1.44",
            board="MATE",
            sensors=["ACC", " gyro ", "BARO", ""],
            motor_count=4,
        )

        self.assertTrue(profile.has_imu)
        self.assertEqual(profile.api_version, "1.44")
        self.assertTrue(profile.supports("baro"))
        self.assertFalse(profile.supports("range"))
        self.assertEqual(profile.sensors, frozenset({"acc", "gyro", "baro"}))

    def test_profile_does_not_assume_optional_sensors(self):
        profile = MachineProfile.from_observation(
            controller="BTFL",
            firmware=None,
            api_version=None,
            board=None,
            sensors=["ACC", "GYRO"],
        )

        self.assertTrue(profile.has_imu)
        self.assertFalse(profile.supports("baro"))
        self.assertFalse(profile.supports("camera"))

    def test_invalid_motor_count_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must be positive"):
            MachineProfile.from_observation(
                controller=None,
                firmware=None,
                api_version=None,
                board=None,
                sensors=[],
                motor_count=0,
            )


if __name__ == "__main__":
    unittest.main()
