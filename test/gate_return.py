import copy
import unittest

import numpy as np

from miniflight.joint import solve_window


class GateReturnTest(unittest.TestCase):
    def test_returning_square_corrects_velocity_and_depth_scale(self):
        rng = np.random.default_rng(7)
        k = np.array([[320., 0, 320], [0, 320., 180], [0, 0, 1.]])
        world = rng.uniform([-3., -2., 8.], [3., 2., 18.], (16, 3))
        gate = np.array([[-.75, -.75, 0], [.75, -.75, 0], [.75, .75, 0], [-.75, .75, 0]])+[.4, 1.6, 12.]
        project = lambda xyz: (xyz @ k.T)[:, :2]/xyz[:, 2:]
        velocity = np.array([.04, -.02, 2.12])
        times = [0., .8, 1., 1.2, 1.4, 1.6, 1.8]
        gravity = np.array([0., 0., 9.81])
        states, frames, edges = [], [], []
        for i, stamp in enumerate(times):
            position = np.array([0., 0., 2.*stamp])
            states.append(np.r_[velocity*stamp, velocity, np.zeros(3)].tolist())
            frames.append(dict(ids=list(range(16)) if i else [],
                pixels=project(world-position).tolist() if i else [],
                gate_pixels=(project(gate-position)+rng.normal(0, .15, (4, 2))).tolist() if stamp >= 1. else []))
            if i:
                dt = stamp-times[i-1]
                edges.append(dict(first=i-1, last=i, dt=dt, dv=(-gravity*dt).tolist(), dp=(-.5*gravity*dt**2).tolist()))
        prior = np.log(world[:, 2]*1.06)
        request = dict(model_version=2, fixed_anchor_position=True, anchor_velocity_sigma=.05,
            intrinsics=k.tolist(), rays=(world/world[:, 2:]).tolist(), depth_prior=prior.tolist(),
            depth_sigma=np.full(16, .5).tolist(), states=states, frames=frames, edges=edges,
            gravity=gravity.tolist(), anchor=states[0][:6], depths=prior.tolist(), gate_world=gate.tolist())
        off = copy.deepcopy(request)
        for frame in off['frames']:
            frame['gate_pixels'] = []
        baseline = solve_window(off)
        corrected = solve_window(request, check_jacobian=True)
        self.assertLess(corrected['jacobian_check_error'], 1e-4)
        for result in (baseline, corrected):
            np.testing.assert_allclose(np.array(result['states'])[0, :3], [0., 0., 0.], atol=1e-9)
        truth = np.array([0., 0., 2.])
        error = lambda result: np.linalg.norm(np.array(result['states'])[-1, 3:6]-truth)
        self.assertLess(error(corrected), .04)
        self.assertLess(error(corrected), error(baseline)*.5)
        self.assertLess(np.linalg.norm(np.array(corrected['states'])[-1, :3]-truth*times[-1]), .08)
        self.assertLess(np.mean(abs(np.exp(corrected['depths'])/world[:, 2]-1)), .02)


if __name__ == '__main__':
    unittest.main()
