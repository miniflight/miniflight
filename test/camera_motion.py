import unittest

import numpy as np

from target.aigp.experiments.camera_imu.motion import GateMotion
from target.aigp.experiments.camera_imu.sensors import Inertial


class MotionTest(unittest.TestCase):
    def test_acceleration_and_dropped_frames(self):
        motion = GateMotion()
        acceleration = np.array([2., -.5, .2])
        velocity = np.array([1., .3, -.1])
        result = None
        for t in [0., .031, .065, .132, .166, .201, .234]:
            result = motion.observe(20.+t, [3., 1., -2.]+velocity*t+.5*acceleration*t*t)
        np.testing.assert_allclose(result["velocity"], velocity+acceleration*(result["time"]-20.), atol=1e-10)
        self.assertLess(result["time"], 20.234)
        self.assertIsNone(motion.observe(20.2, np.zeros(3)))

    def test_delayed_corrections_reach_present(self):
        imu = Inertial()
        imu.times = [0., .1, .2, .3]
        imu.positions = [np.array([t, 0., 0.]) for t in imu.times]
        imu.velocities = [np.array([1., 0., 0.]) for _ in imu.times]
        imu.rotations = [np.eye(3) for _ in imu.times]
        imu.position, imu.velocity = imu.positions[-1].copy(), imu.velocities[-1].copy()
        imu.correct_velocity(.1, [2., 0., 0.], 1.)
        imu.correct_position(.2, [.6, 0., 0.], 1.)
        np.testing.assert_allclose(imu.position, [.8, 0., 0.])
        np.testing.assert_allclose(imu.velocity, [2., 0., 0.])
        np.testing.assert_allclose(imu.positions[0], np.zeros(3))


if __name__ == "__main__":
    unittest.main()
