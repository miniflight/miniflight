import unittest

import numpy as np

from miniflight.odometry import ImuIntegral
from miniflight.joint import solve_window
from target.aigp.experiments.camera_imu.joint_control import reference, slow_reference


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


class JointTest(unittest.TestCase):
    def test_slow_profile_decelerates_before_turn(self):
        start, forward = np.zeros(3), np.array([1., 0., 0.])
        angle = np.radians(20.)
        for t in np.linspace(1., 2.99, 10):
            self.assertAlmostEqual(np.linalg.norm(slow_reference(start,forward,2.,angle,t)[1]),1.)
        for boundary in (1.,3.,4.):
            a=slow_reference(start,forward,2.,angle,boundary-1e-7)
            b=slow_reference(start,forward,2.,angle,boundary+1e-7)
            np.testing.assert_allclose(a[:2],b[:2],atol=1e-6)
        end=slow_reference(start,forward,2.,angle,5.)
        self.assertLess(np.linalg.norm(end[0]),4.1)
        np.testing.assert_allclose(end[1:],0.,atol=1e-10)

    def test_slow_turn_reference_is_continuous_and_stops(self):
        start, forward = np.zeros(3), np.array([1., 0., 0.])
        angle = np.radians(20.)
        for t in [0., .5, 1., 1.999]:
            p, v, a = reference(start, forward, 2., angle, 2., 1.5, t)
            self.assertAlmostEqual(np.linalg.norm(v), 2.)
            self.assertAlmostEqual(p[2], 0.)
        before = reference(start, forward, 2., angle, 2., 1.5, 2.-1e-6)
        after = reference(start, forward, 2., angle, 2., 1.5, 2.+1e-6)
        np.testing.assert_allclose(before[0], after[0], atol=1e-5)
        np.testing.assert_allclose(before[1], after[1], atol=1e-5)
        end = reference(start, forward, 2., angle, 2., 1.5, 4.)
        np.testing.assert_allclose(end[1:], 0., atol=1e-10)
        self.assertGreater(end[0][1], .5)

    @staticmethod
    def problem(acceleration=.6, depth_scale=1.1):
        rng = np.random.default_rng(3)
        k = np.array([[320., 0, 320], [0, 320, 180], [0, 0, 1.]])
        world = rng.uniform([-2., -1., 5.], [2., 1., 12.], (16, 3))
        prior = np.log(world[:, 2]*depth_scale)
        gravity = np.array([0., 9.81, 0.])
        states, frames, edges = [], [], []
        for i, t in enumerate(np.arange(5)*.1):
            p = np.array([.5*acceleration*t*t, 0., 1.5+2*t])
            states.append(np.r_[[0., 0., 1.5+2*t], [0., 0., 2.], [0., 0., 0.]].tolist())
            camera = world-p
            h = camera @ k.T
            frames.append(dict(ids=list(range(16)) if i else [],
                               pixels=(h[:, :2]/h[:, 2:]).tolist() if i else []))
            if i:
                force = np.array([acceleration, 0., 0.])-gravity
                edges.append(dict(first=i-1, last=i, dt=.1, dv=(force*.1).tolist(), dp=(force*.005).tolist()))
        return dict(intrinsics=k.tolist(), rays=(world/world[:, 2:]).tolist(), depth_prior=prior.tolist(),
            depth_sigma=np.full(16, .2).tolist(), states=states, frames=frames, edges=edges,
            gravity=gravity.tolist(), anchor=states[0][:6], depths=prior.tolist()), world

    def test_acceleration_updates_velocity_and_depth(self):
        request, world = self.problem()
        result = solve_window(request, check_jacobian=True)
        last = np.array(result['states'])[-1]
        self.assertTrue(result['finite'])
        self.assertLess(result['jacobian_check_error'], 1e-4)
        np.testing.assert_allclose(last[:3], [.048, 0., 2.3], atol=.07)
        np.testing.assert_allclose(last[3:6], [.24, 0., 2.], atol=.08)
        self.assertLess(np.mean(abs(np.exp(result['depths'])/world[:, 2]-1)), .05)
        self.assertLess(result['reprojection_rms_px'], .1)

    def test_coasting_with_gravity_stays_consistent(self):
        request, world = self.problem(0., 1.)
        result = solve_window(request)
        np.testing.assert_allclose(result['states'], request['states'], atol=1e-6)
        np.testing.assert_allclose(np.exp(result['depths']), world[:, 2], atol=1e-6)


if __name__ == '__main__':
    unittest.main()
