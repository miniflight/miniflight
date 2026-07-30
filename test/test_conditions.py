"""Tests for supervisor facts derived from one sensor tick."""

import unittest

from miniflight.core.conditions import ConditionSet
from miniflight.core.observation import ObservationTick


def tick(**changes):
    values = {
        "sequence": 1,
        "timestamp_ns": 2_000,
        "accel_mps2": (0.0, 0.0, -9.81),
        "gyro_rad_s": (0.0, 0.0, 0.0),
        "source_age_ns": 100,
    }
    values.update(changes)
    return ObservationTick(**values)


class ConditionSetTests(unittest.TestCase):
    def test_fresh_imu_and_camera_enable_visual_hold(self):
        conditions = ConditionSet.from_tick(
            tick(
                camera_data=b"pixels",
                camera_timestamp_ns=1_900,
                camera_width=2,
                camera_height=1,
                camera_encoding="rgb8",
            ),
            link_live=True,
            max_imu_age_ns=100,
            max_camera_age_ns=100,
        )

        self.assertTrue(conditions.imu_fresh)
        self.assertTrue(conditions.camera_fresh)
        self.assertTrue(conditions.visual_hold_ready)

    def test_missing_camera_does_not_enable_visual_hold(self):
        conditions = ConditionSet.from_tick(
            tick(),
            link_live=True,
            max_imu_age_ns=100,
            max_camera_age_ns=100,
        )

        self.assertFalse(conditions.camera_present)
        self.assertFalse(conditions.camera_fresh)
        self.assertFalse(conditions.visual_hold_ready)

    def test_dead_link_makes_all_freshness_false(self):
        conditions = ConditionSet.from_tick(
            tick(),
            link_live=False,
            max_imu_age_ns=100,
            max_camera_age_ns=100,
        )

        self.assertFalse(conditions.imu_fresh)
        self.assertFalse(conditions.visual_hold_ready)

    def test_negative_limits_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "non-negative"):
            ConditionSet.from_tick(
                tick(),
                link_live=True,
                max_imu_age_ns=-1,
                max_camera_age_ns=100,
            )


if __name__ == "__main__":
    unittest.main()
