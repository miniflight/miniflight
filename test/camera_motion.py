import unittest
import json
from pathlib import Path
import tempfile

import numpy as np

from target.aigp.experiments.camera_imu.motion import GateMotion
from target.aigp.experiments.camera_imu.sensors import Inertial
from target.aigp.experiments.camera_imu import vision
from scipy.spatial.transform import Rotation
from miniflight.state import ImuIncrement
from target.aigp.experiments.camera_imu.accuracy import score_estimates


class MotionTest(unittest.TestCase):
    def test_edges_between_pixel_centers(self):
        axis = np.arange(128)
        for phase in np.linspace(.05, .95, 10):
            x, y, side = 25.+phase, 25.+phase*.7, 70.5
            coverage = lambda a, b: np.maximum(0, np.minimum(axis+.5, b)-np.maximum(axis-.5, a))
            inner = np.outer(coverage(y, y+side), coverage(x, x+side))
            outer = np.outer(coverage(y-5, y+side+5), coverage(x-5, x+side+5))
            image = np.zeros((128, 128, 3), dtype=np.uint8)
            image[:, :, 2] = np.round(255*(outer-inner)).astype(np.uint8)
            corners = np.array([[x, y], [x+side, y], [x+side, y+side], [x, y+side]])
            found = vision.refine(image, corners+[.13, .17])
            self.assertLess(float(np.sqrt(np.mean((found-corners)**2))), .1)

    def test_scoring_frame_times_and_json(self):
        rotation = Rotation.from_euler("z", .7).as_matrix()
        center, anchor = np.array([4., 5., 6.]), np.array([2., 3., 4.])
        times = np.arange(0., 1.01, .01)
        speed = np.array([1., 0., 0.])
        relative = np.column_stack((times, np.zeros(len(times)), -np.ones(len(times))))
        truth = np.column_stack((times, relative @ rotation.T+center, np.tile(rotation @ speed, (len(times), 1))))
        tick = dict(imu_time=.6, position=(anchor+[.6, 0., -1.]).tolist(), velocity=speed,
                    gate_center=anchor, unfused_position=anchor+[.7, 0., -1.], unfused_velocity=[1.2, 0., 0.])
        observed = dict(image_id=1, capture_time=.5, position_relative_gate=[.5, 0., -1.],
            velocity=dict(time=.4, velocity=speed, window_s=.2, observations=7), capture_to_output_s=.03,
            correction_enabled=True, position_correction=[.1, 0., 0.], velocity_correction=[.2, 0., 0.])
        with tempfile.TemporaryDirectory() as directory:
            scored = score_estimates(Path(directory), truth, [tick], center, rotation, [observed])
        self.assertLess(scored["fused_position_m"]["rms"], 1e-10)
        self.assertAlmostEqual(scored["unfused_position_m"]["rms"], .1)
        self.assertAlmostEqual(scored["unfused_velocity_m_s"]["rms"], .2)
        self.assertAlmostEqual(scored["capture_to_output_ms"]["median"], 30.)
        self.assertAlmostEqual(scored["velocity_fit_age_ms"]["median"], 100.)
        json.dumps(scored)

    def test_correction_leaves_unfused_baseline_unchanged(self):
        imu = Inertial()
        for i in range(101):
            imu.update([ImuIncrement(i*.01, .01, (0. if i <= 50 else 1.2, 0., -9.81), (0., 0., 0.))])
        imu.correct_velocity(.7, [.2, 0., 0.], 1.)
        imu.correct_position(.9, [.08, 0., 0.], 1.)
        np.testing.assert_allclose(imu.raw_position, [.15, 0., 0.], atol=1e-10)
        np.testing.assert_allclose(imu.raw_velocity, [.6, 0., 0.], atol=1e-10)
        np.testing.assert_allclose(imu.position, [.13, 0., 0.], atol=1e-10)
        np.testing.assert_allclose(imu.velocity, [.56, 0., 0.], atol=1e-10)

    def test_camera_translation_with_independent_rotation(self):
        motion = GateMotion()
        center, speed = np.array([.3, -.2, 6.]), np.array([.5, -.1, .2])
        for t in np.arange(0., .31, 1/30):
            rotation = Rotation.from_euler("xyz", [t*.1, t*.3, -t*.2]).as_matrix()
            camera = (vision.SQUARE+center-speed*t) @ rotation
            pixels = camera @ vision.K.T
            pose = vision.best_pose(pixels[:, :2]/pixels[:, 2:])
            measured = -rotation @ pose[2]
            np.testing.assert_allclose(measured, speed*t-center, atol=1e-6)
            observed = motion.observe(t, measured)
        np.testing.assert_allclose(observed["velocity"], speed, atol=1e-5)

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
