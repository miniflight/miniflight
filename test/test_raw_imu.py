"""Tests for unscaled IMU transport samples."""

import unittest

from miniflight.core.raw_imu import RawImuSample


class RawImuSampleTests(unittest.TestCase):
    def test_sample_preserves_raw_vectors(self):
        sample = RawImuSample(2, 20, (1.0, 2.0, 3.0), (-1.0, 0.0, 1.0), 4)

        self.assertEqual(sample.accel_raw, (1.0, 2.0, 3.0))
        self.assertEqual(sample.gyro_raw, (-1.0, 0.0, 1.0))

    def test_invalid_raw_value_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            RawImuSample(0, 0, (0.0, 0.0, float("nan")), (0.0, 0.0, 0.0), 0)


if __name__ == "__main__":
    unittest.main()
