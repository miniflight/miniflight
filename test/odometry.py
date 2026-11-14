import unittest

import numpy as np

from miniflight.odometry import ImuIntegral


class IntegralTest(unittest.TestCase):
    def test_measured_dt_with_bursty_publication(self):
        integral = ImuIntegral()
        force = np.array([.3, -.2, -9.81])
        for stamp, dt in [(1., .01), (1.001, .02), (1.04, .015), (1.041, .025)]:
            integral.append(stamp, dt, force)
        duration, velocity, position = integral.between(1., 1.041)
        self.assertAlmostEqual(duration, .06)
        np.testing.assert_allclose(velocity, force*.06, atol=1e-12)
        np.testing.assert_allclose(position, .5*force*.06**2, atol=1e-12)

    def test_gravity_cancels_without_camera_motion(self):
        integral = ImuIntegral()
        gravity = np.array([.5, 3., 9.3])
        for i in range(1, 101):
            integral.append(i*.01, .01, -gravity)
        duration, velocity, position = integral.between(.2, .8)
        np.testing.assert_allclose(velocity+gravity*duration, 0., atol=1e-10)
        np.testing.assert_allclose(position+.5*gravity*duration**2, 0., atol=1e-10)

    def test_missing_or_regressing_history_rejected(self):
        integral = ImuIntegral()
        with self.assertRaises(ValueError):
            integral.at(1.)
        integral.append(1., .01, [0., 0., 0.])
        with self.assertRaises(ValueError):
            integral.append(.9, .01, [0., 0., 0.])


if __name__ == '__main__':
    unittest.main()
