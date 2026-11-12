import unittest

import numpy as np

from target.aigp.experiments.patch_track import seeds, track


class PatchTest(unittest.TestCase):
    @staticmethod
    def image(dx=0., dy=0.):
        y, x = np.mgrid[:64, :64].astype(float)
        x, y = x-dx, y-dy
        return 100+30*np.sin(.35*x)+25*np.cos(.41*y)+15*np.sin(.25*x+.31*y)

    def test_known_fractional_translation(self):
        result = track([self.image(), self.image(.35, -.2)], [32., 32.])[-1]
        self.assertEqual(result["status"], "tracked")
        np.testing.assert_allclose(result["point"], [32.35, 31.8], atol=.15)

    def test_seed_coverage_and_blank(self):
        image = np.zeros((120, 160), dtype=np.uint8)
        self.assertEqual(len(seeds(image)), 0)
        for y in range(3):
            for x in range(4):
                image[y*40+18:y*40+25, x*40+18:x*40+25] = 200
        occupied = {(int(x//40), int(y//40)) for x, y in seeds(image)}
        self.assertEqual(len(occupied), 12)

    def test_reject_missing_information(self):
        flat = np.full((64, 64), 80.)
        edge = np.tile(np.r_[np.zeros(32), np.full(32, 100.)], (64, 1))
        self.assertEqual(track([flat, flat], [32., 32.])[0]["status"], "lost_low_contrast")
        self.assertEqual(track([edge, edge], [32., 32.])[0]["status"], "lost_one_dimensional")
        self.assertNotEqual(track([self.image(), flat], [32., 32.])[-1]["status"], "tracked")

    def test_stop_on_appearance_change(self):
        rows = track([self.image(), self.image()+70, self.image()], [32., 32.])
        self.assertEqual(len(rows), 2)
        self.assertNotEqual(rows[-1]["status"], "tracked")

    def test_outside_image(self):
        self.assertEqual(track([self.image()], [2., 2.])[0]["status"], "lost_outside_image")

    def test_sequence_and_independent_reverse(self):
        images = [self.image(i*.16, -i*.23) for i in range(40)]
        for anchor in (False, True):
            with self.subTest(anchor_first=anchor):
                forward = track(images, [32., 32.], anchor_first=anchor)
                self.assertTrue(all(row["status"] == "tracked" for row in forward))
                np.testing.assert_allclose(forward[-1]["point"], [38.24, 23.03], atol=.2)
                backward = track(images[::-1], forward[-1]["point"], anchor_first=anchor)
                self.assertTrue(all(row["status"] == "tracked" for row in backward))
                np.testing.assert_allclose(backward[-1]["point"], [32., 32.], atol=.2)


if __name__ == "__main__":
    unittest.main()
