"""Tests for safe declarative bench-test plans."""

import unittest

from miniflight.core.bench import BenchPlan


class BenchPlanTests(unittest.TestCase):
    def test_valid_plan_describes_one_bounded_output_test(self):
        plan = BenchPlan(
            output_index=0,
            command_fraction=0.1,
            duration_ms=100,
            expected_observation="motor vibration on front-right arm",
        )

        self.assertEqual(plan.output_index, 0)
        self.assertEqual(plan.duration_ms, 100)

    def test_long_or_unbounded_test_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "duration_ms"):
            BenchPlan(0, 0.1, 501, "motor vibration")

    def test_invalid_output_fraction_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "command_fraction"):
            BenchPlan(0, 0.0, 100, "motor vibration")

    def test_missing_observation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "expected_observation"):
            BenchPlan(0, 0.1, 100, " ")


if __name__ == "__main__":
    unittest.main()
