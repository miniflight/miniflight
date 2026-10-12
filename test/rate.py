import unittest

from miniflight.rate import rate_control


class RateTest(unittest.TestCase):
    def test_integral_depends_on_elapsed_time_not_call_count(self):
        kp, ki, target, measured = (.04,) * 3, (.2,) * 3, (.3, -.2, .1), (0, 0, 0)
        integral = (0, 0, 0)
        for dt in (.02, 0, .03, 0, .05):
            torque, integral = rate_control(kp, ki, integral, target, measured, dt)
        one_torque, one_integral = rate_control(kp, ki, (0, 0, 0), target, measured, .1)
        for actual, expected in zip((*torque, *integral), (*one_torque, *one_integral)):
            self.assertAlmostEqual(actual, expected)
        _, unchanged = rate_control(kp, ki, integral, target, measured, 0)
        self.assertEqual(unchanged, integral)

    def test_invalid_sample_or_clock_is_rejected(self):
        for measured, dt in (((0, 0, 0), -.01), ((0, 0, float("nan")), .02)):
            with self.subTest(measured=measured, dt=dt), self.assertRaises(ValueError):
                rate_control((.04,) * 3, (.2,) * 3, (0, 0, 0), (0, 0, 0), measured, dt)
