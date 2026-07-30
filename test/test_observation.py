"""Tests for one canonical Miniflight sensor tick."""

import unittest

from miniflight.core.observation import ObservationTick


def imu_tick(**changes):
    values = {
        "sequence": 1,
        "timestamp_ns": 2_000,
        "accel_mps2": (0.0, 0.0, -9.81),
        "gyro_rad_s": (0.0, 0.0, 0.0),
        "source_age_ns": 100,
    }
    values.update(changes)
    return ObservationTick(**values)


class ObservationTickTests(unittest.TestCase):
    def test_imu_only_tick_is_valid(self):
        tick = imu_tick()

        self.assertFalse(tick.has_camera)
        self.assertEqual(tick.sequence, 1)

    def test_complete_camera_sample_is_valid(self):
        tick = imu_tick(
            camera_data=b"pixels",
            camera_timestamp_ns=1_900,
            camera_width=2,
            camera_height=1,
            camera_encoding="rgb8",
        )

        self.assertTrue(tick.has_camera)

    def test_incomplete_camera_sample_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "camera data requires"):
            imu_tick(camera_data=b"pixels")

    def test_future_camera_sample_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "newer than the tick"):
            imu_tick(
                camera_data=b"pixels",
                camera_timestamp_ns=2_001,
                camera_width=2,
                camera_height=1,
                camera_encoding="rgb8",
            )

    def test_non_finite_imu_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            imu_tick(gyro_rad_s=(0.0, float("nan"), 0.0))


if __name__ == "__main__":
    unittest.main()
