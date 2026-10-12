import unittest
from pathlib import Path

import numpy as np

from miniflight.reference import sample_motion


class MotionReferenceTest(unittest.TestCase):
    def test_saved_r1_reference_crosses_its_six_gate_centers(self):
        for name, duration in (("vq1_r1_35s", 35), ("vq1_r1_lift_35s", 35),
                               ("vq1_r1_lift_32s", 32), ("vq1_r1_lift_30s", 30)):
            with self.subTest(reference=name), np.load(Path(__file__).parent / f"fixtures/{name}.npz") as plan:
                states, rates, dt = plan["states"], plan["acceleration_rates"], float(plan["dt"])
                self.assertEqual(len(plan["gates"]), 6)
                self.assertEqual(len(rates) * dt, duration)
                for index, center in zip(plan["gates"], plan["points"][1:]):
                    position, _, _ = sample_motion(states, rates, dt, index * dt)
                    np.testing.assert_allclose(position, center, atol=1e-8)

    def test_derivatives_and_continuity_through_interval_boundaries(self):
        states = np.array([[[0, 0, 0], [1, 0, 0], [0, 0, 0]],
                           [[7/6, 0, 0], [1.5, 0, 0], [1, 0, 0]],
                           [[3, 0, 0], [2, 0, 0], [0, 0, 0]]])
        rates = np.array([[1, 0, 0], [-1, 0, 0]])
        for time in (.4, 1.0, 1.4, 2.0, 2.4):
            with self.subTest(time=time):
                position, velocity, acceleration = sample_motion(states, rates, 1, time)
                before = sample_motion(states, rates, 1, time - 1e-5)
                after = sample_motion(states, rates, 1, time + 1e-5)
                np.testing.assert_allclose((after[0] - before[0]) / 2e-5, velocity, atol=1e-8)
                np.testing.assert_allclose((after[1] - before[1]) / 2e-5, acceleration, atol=1e-5)
        np.testing.assert_allclose(sample_motion(states, rates, 1, 2), states[-1])


if __name__ == "__main__":
    unittest.main()
