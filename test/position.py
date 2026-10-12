import unittest

from miniflight.position import trajectory_acceleration


class TrajectoryControlTest(unittest.TestCase):
    def test_matching_motion_preserves_feedforward_and_integral(self):
        position, velocity, acceleration = (12, -7, 3), (4, -2, 1), (.4, 0, -.2)
        integral = (.1, -.1, 0)
        request, next_integral = trajectory_acceleration(
            position, velocity, (position, velocity, acceleration), integral, .02,
            kp=2, ki=1, kd=2, integral_limit=3)
        self.assertEqual(request, (.5, -.1, -.2))
        self.assertEqual(next_integral, integral)

    def test_integral_saturates_and_unwinds_without_target_state(self):
        integral = (0, 0, 0)
        for target, expected in (((10, 0, 0), (1, 0, 0)), ((-1, 0, 0), (0, 0, 0))):
            _, integral = trajectory_acceleration(
                (0, 0, 0), (0, 0, 0), (target, (0, 0, 0), (0, 0, 0)), integral, 1,
                kp=0, ki=1, kd=0, integral_limit=1)
            self.assertEqual(integral, expected)
