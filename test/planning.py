import unittest
from pathlib import Path

import numpy as np

from target.aigp.experiments.plan_motion import plan


class MotionPlanningTest(unittest.TestCase):
    def test_lift_and_gate_constraints_at_both_paces(self):
        with np.load(Path(__file__).parent / "fixtures/vq1_r1_35s.npz") as source:
            for duration in (35, 32):
                result = plan(source, duration)
                states, jerk, dt = result["states"], result["acceleration_rates"], result["dt"]
                np.testing.assert_array_equal(states[0], [[0, 0, 0], [0, 0, 0], [0, 0, -2]])
                np.testing.assert_allclose(states[result["gates"], 0], result["points"][1:], atol=1e-8)
                np.testing.assert_allclose(states[1:, 0], states[:-1, 0] + states[:-1, 1]*dt + states[:-1, 2]*dt**2/2 + jerk*dt**3/6, atol=1e-8)
                np.testing.assert_allclose(states[1:, 1], states[:-1, 1] + states[:-1, 2]*dt + jerk*dt**2/2, atol=1e-8)
                np.testing.assert_allclose(states[1:, 2], states[:-1, 2] + jerk*dt, atol=1e-8)
                self.assertLessEqual(abs(states[:, 2]).max(), 2.1 + 1e-8)
                self.assertLessEqual(abs(jerk).max(), 4 + 1e-8)
