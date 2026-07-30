"""Tests for the IMU stillness gate."""

import unittest

from miniflight.core.observation import ObservationTick
from miniflight.core.stillness import StillnessDetector


def tick(sequence, accel=(0.0, 0.0, -9.80665), gyro=(0.0, 0.0, 0.0)):
    return ObservationTick(
        sequence=sequence,
        timestamp_ns=sequence * 1_000,
        accel_mps2=accel,
        gyro_rad_s=gyro,
        source_age_ns=0,
    )


class StillnessDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = StillnessDetector(
            required_samples=3,
            max_gyro_rad_s=0.1,
            max_accel_error_mps2=0.2,
        )

    def test_requires_a_consecutive_quiet_window(self):
        self.assertFalse(self.detector.add(tick(1)))
        self.assertFalse(self.detector.add(tick(2)))
        self.assertTrue(self.detector.add(tick(3)))
        self.assertEqual(self.detector.accepted_samples, 3)

    def test_rotation_resets_the_quiet_window(self):
        self.detector.add(tick(1))
        self.detector.add(tick(2, gyro=(0.2, 0.0, 0.0)))

        self.assertEqual(self.detector.accepted_samples, 0)
        self.assertFalse(self.detector.add(tick(3)))

    def test_acceleration_error_resets_the_quiet_window(self):
        self.detector.add(tick(1))
        self.detector.add(tick(2, accel=(0.0, 0.0, -9.4)))

        self.assertEqual(self.detector.accepted_samples, 0)

    def test_reused_tick_is_rejected(self):
        self.detector.add(tick(1))

        with self.assertRaisesRegex(RuntimeError, "sequence must increase"):
            self.detector.add(tick(1))


if __name__ == "__main__":
    unittest.main()
