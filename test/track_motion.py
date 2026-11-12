import unittest

import cv2
import numpy as np

from miniflight.vision import translation_direction, triangulate_views


class TranslationTest(unittest.TestCase):
    def test_multiview_depth_with_pixel_noise(self):
        rng = np.random.default_rng(3)
        k = np.array([[320., 0, 320], [0, 320, 180], [0, 0, 1]])
        times = np.linspace(0, 1, 12)
        positions = np.column_stack((times*1.5, times*.3, times*3))
        rotations = np.array([cv2.Rodrigues(np.array([t*.03, -t*.05, t*.02]))[0] for t in times])
        errors = [[], []]
        for point in rng.uniform([-4, -3, 10], [4, 3, 60], (100, 3)):
            camera = np.einsum('nij,nj->ni', rotations, point-positions)
            projected = camera @ k.T
            pixels = projected[:, :2]/projected[:, 2:]
            np.testing.assert_allclose(triangulate_views(pixels, rotations, positions, k), point, atol=1e-7)
            noisy = pixels+rng.normal(0, .2, pixels.shape)
            for case, indices in enumerate(([0, 11], np.arange(12))):
                estimated = triangulate_views(noisy[indices], rotations[indices], positions[indices], k)
                self.assertIsNotNone(estimated)
                errors[case].append(np.linalg.norm(estimated-point))
        rms = [np.sqrt(np.mean(np.array(error)**2)) for error in errors]
        self.assertLess(rms[1], .8*rms[0])

    def test_no_translation_has_no_depth(self):
        rotations = np.array([cv2.Rodrigues(np.array([0., t*.05, 0.]))[0] for t in range(4)])
        camera = rotations @ np.array([1., .5, 8.])
        pixels = camera[:, :2]/camera[:, 2:]
        self.assertIsNone(triangulate_views(pixels, rotations, np.zeros((4, 3)), np.eye(3)))

    def test_translation_with_rotation_and_depth(self):
        rng = np.random.default_rng(4)
        world = rng.uniform([-2., -1.5, 3.], [2., 1.5, 12.], (60, 3))
        intrinsics = np.array([[320., 0, 320], [0, 320, 180], [0, 0, 1]])
        rotation, _ = cv2.Rodrigues(np.array([.02, -.03, .01]))

        def project(points):
            image = points @ intrinsics.T
            return image[:, :2]/image[:, 2:]

        for displacement in ([.2, -.1, .6], [-.2, .1, -.6]):
            with self.subTest(displacement=displacement):
                expected = np.array(displacement)/np.linalg.norm(displacement)
                first, second = project(world), project((world-displacement) @ rotation.T)
                direction, positive, _ = translation_direction(first, second, intrinsics, rotation)
                np.testing.assert_allclose(direction, expected, atol=1e-10)
                self.assertTrue(positive.all())
        self.assertIsNone(translation_direction(project(world), project(world @ rotation.T), intrinsics, rotation))

    def test_missing_constraints(self):
        points = np.array([[1., 2.], [3., 4.]])
        self.assertIsNone(translation_direction(points, points, np.eye(3), np.eye(3)))


if __name__ == '__main__':
    unittest.main()
