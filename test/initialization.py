import unittest

import cv2
import numpy as np

from miniflight.initialization import fit_gate_motion


class GateInitializationTest(unittest.TestCase):
    def test_joint_gate_motion_with_imu_acceleration(self):
        k = np.array([[320., 0, 320], [0, 320, 180], [0, 0, 1.]])
        square = .75*np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]])
        times = np.linspace(0, 2., 48)
        velocity = np.array([.08, -.03, 2.])
        offsets = .5*times[:, None]**2*np.array([.12, -.04, .08])
        center = np.array([.4, 2.4, 13.])
        angle = np.array([3.05, .03, -.02])
        gate = square @ cv2.Rodrigues(angle)[0].T+center
        rotations = np.array([cv2.Rodrigues(np.array([t*.01, -t*.015, t*.006]))[0] for t in times])
        camera = np.einsum('nij,nkj->nki', rotations, gate[None]-times[:,None,None]*velocity-offsets[:,None])
        h = camera @ k.T
        pixels = h[:,:,:2]/h[:,:,2:]
        request = dict(intrinsics=k.tolist(), square=square.tolist(), times=times.tolist(),
            rotations=rotations.tolist(), offsets=offsets.tolist(), pixels=pixels.tolist(),
            seeds=[dict(center=(center+[.1,-.1,.5]).tolist(), rotation=(angle+[.01,.02,-.01]).tolist(),
                        velocity=(velocity+[.1,.1,-.1]).tolist())])
        result = fit_gate_motion(request)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['velocity'], velocity, atol=1e-5)
        np.testing.assert_allclose(result['center'], center, atol=1e-5)
        rng = np.random.default_rng(12)
        request['pixels'] = (pixels+rng.normal(0,.12,pixels.shape)).tolist()
        result = fit_gate_motion(request)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(np.array(result['velocity'])-velocity), .04)


if __name__ == '__main__':
    unittest.main()
