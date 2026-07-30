"""Tests for raw IMU conversion into Miniflight canonical units."""

import math
import unittest

from miniflight.core.imu import ImuCalibration


def calibration(**changes):
    values = {
        "accel_scale_mps2_per_unit": 9.81,
        "gyro_scale_rad_s_per_unit": math.pi / 180.0,
        "accel_bias_raw": (0.0, 0.0, 0.0),
        "gyro_bias_raw": (0.0, 0.0, 0.0),
        "body_from_sensor": (1, 2, 3),
    }
    values.update(changes)
    return ImuCalibration(**values)


class ImuCalibrationTests(unittest.TestCase):
    def test_identity_conversion_scales_to_si_units(self):
        accel, gyro = calibration().apply((1.0, 0.0, -1.0), (180.0, 0.0, 0.0))

        self.assertEqual(accel, (9.81, 0.0, -9.81))
        self.assertAlmostEqual(gyro[0], math.pi)

    def test_bias_and_axis_permutation_are_applied(self):
        calib = calibration(
            accel_scale_mps2_per_unit=1.0,
            gyro_scale_rad_s_per_unit=1.0,
            accel_bias_raw=(1.0, 2.0, 3.0),
            gyro_bias_raw=(0.0, 1.0, 0.0),
            body_from_sensor=(2, -1, 3),
        )

        accel, gyro = calib.apply((11.0, 22.0, 33.0), (10.0, 21.0, 30.0))

        self.assertEqual(accel, (20.0, -10.0, 30.0))
        self.assertEqual(gyro, (20.0, -10.0, 30.0))

    def test_repeated_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "signed axis permutation"):
            calibration(body_from_sensor=(1, 1, 3))

    def test_nonpositive_scale_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "scales must be positive"):
            calibration(accel_scale_mps2_per_unit=0.0)


if __name__ == "__main__":
    unittest.main()
