import unittest

import cv2
import numpy as np

from miniflight.vision import translation_direction


class TranslationTest(unittest.TestCase):
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
