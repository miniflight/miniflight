"""Tests for stationary raw gyro-bias estimation."""

import unittest

from miniflight.core.bias import GyroBiasEstimator


class GyroBiasEstimatorTests(unittest.TestCase):
    def test_estimator_averages_and_freezes_required_samples(self):
        estimator = GyroBiasEstimator(required_samples=3)

        self.assertFalse(estimator.add(1, (1.0, 2.0, 3.0)))
        self.assertFalse(estimator.add(2, (2.0, 3.0, 4.0)))
        self.assertTrue(estimator.add(3, (3.0, 4.0, 5.0)))

        self.assertEqual(estimator.bias_raw, (2.0, 3.0, 4.0))

    def test_extra_samples_do_not_change_frozen_bias(self):
        estimator = GyroBiasEstimator(required_samples=1)
        estimator.add(1, (1.0, 2.0, 3.0))
        estimator.add(2, (100.0, 200.0, 300.0))

        self.assertEqual(estimator.bias_raw, (1.0, 2.0, 3.0))
        self.assertEqual(estimator.sample_count, 1)

    def test_bias_is_unavailable_before_window_completes(self):
        estimator = GyroBiasEstimator(required_samples=2)
        estimator.add(1, (1.0, 2.0, 3.0))

        with self.assertRaisesRegex(RuntimeError, "not ready"):
            _ = estimator.bias_raw

    def test_reused_sequence_is_rejected(self):
        estimator = GyroBiasEstimator(required_samples=2)
        estimator.add(1, (0.0, 0.0, 0.0))

        with self.assertRaisesRegex(RuntimeError, "sequence must increase"):
            estimator.add(1, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
