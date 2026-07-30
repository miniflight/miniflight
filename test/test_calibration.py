"""Tests for passive stationary IMU reference estimation."""

import unittest

from miniflight.calibration import STANDARD_GRAVITY_MPS2, StationaryImuReferenceEstimator
from miniflight.core.raw_imu import RawImuSample


def sample(sequence, accel=(0.0, 0.0, 2.0), gyro=(0.0, 0.0, 0.0)):
    return RawImuSample(sequence, sequence, accel, gyro, 0)


class StationaryImuReferenceEstimatorTests(unittest.TestCase):
    def test_reference_uses_the_admitted_window_mean(self):
        estimator = StationaryImuReferenceEstimator(2)

        self.assertFalse(estimator.add(sample(1, (0.0, 0.0, 2.0), (1.0, 2.0, 3.0))))
        self.assertTrue(estimator.add(sample(2, (0.0, 0.0, 4.0), (3.0, 4.0, 5.0))))

        self.assertEqual(estimator.reference.sample_count, 2)
        self.assertEqual(estimator.reference.accel_mean_raw, (0.0, 0.0, 3.0))
        self.assertEqual(estimator.reference.gyro_bias_raw, (2.0, 3.0, 4.0))
        self.assertAlmostEqual(estimator.reference.accel_scale_mps2_per_count, STANDARD_GRAVITY_MPS2 / 3.0)

    def test_duplicate_sequences_are_rejected(self):
        estimator = StationaryImuReferenceEstimator(2)
        estimator.add(sample(2))

        with self.assertRaisesRegex(RuntimeError, "sequence"):
            estimator.add(sample(2))

    def test_zero_stationary_acceleration_is_rejected(self):
        estimator = StationaryImuReferenceEstimator(1)

        with self.assertRaisesRegex(RuntimeError, "magnitude"):
            estimator.add(sample(1, (0.0, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
