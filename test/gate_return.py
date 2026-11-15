import copy
import unittest

import numpy as np
import cv2

from miniflight.joint import solve_window
from target.aigp.experiments.camera_imu.vision import mapped_aperture


class GateReturnTest(unittest.TestCase):
    def test_unpaired_capture_does_not_fabricate_a_scoring_view(self):
        import struct
        import tempfile
        from pathlib import Path
        from target.aigp.experiments.camera_imu.joint_score import rendered_views
        for identity in (0,17):
            blob=bytearray(4096+2*512)
            struct.pack_into('<Q',blob,0,1)
            record=[0]*16;record[0]=8;record[11]=identity;record[15]=1
            struct.pack_into('<16Q',blob,4096,*record)
            with tempfile.TemporaryDirectory() as temporary:
                path=Path(temporary)/'events.bin';path.write_bytes(blob)
                self.assertEqual(rendered_views(path),{})

    def test_redetection_recovers_shifted_prediction_and_preserves_identity(self):
        image = np.zeros((360,640,3),np.uint8)
        cv2.rectangle(image,(200,130),(360,290),(0,0,220),-1)
        cv2.rectangle(image,(220,150),(340,270),(0,0,0),-1)
        cv2.rectangle(image,(430,90),(600,260),(0,0,220),-1)
        cv2.rectangle(image,(450,110),(580,240),(0,0,0),-1)
        predicted = np.array([[220.,150.],[340.,150.],[340.,270.],[220.,270.]])+[24.,0.]
        corners,source,_ = mapped_aperture(image,predicted)
        self.assertEqual(source,'redetected')
        np.testing.assert_allclose(corners.mean(axis=0),[280.,210.],atol=.6)
        image[120:301,190:226] = 0
        corners,_,_ = mapped_aperture(image,predicted)
        self.assertIsNone(corners)

    @staticmethod
    def problem(times=None,last_gate_time=float('inf')):
        rng = np.random.default_rng(7)
        k = np.array([[320., 0, 320], [0, 320., 180], [0, 0, 1.]])
        world = rng.uniform([-3., -2., 8.], [3., 2., 18.], (16, 3))
        gate = np.array([[-.75, -.75, 0], [.75, -.75, 0], [.75, .75, 0], [-.75, .75, 0]])+[.4, 1.6, 12.]
        project = lambda xyz: (xyz @ k.T)[:, :2]/xyz[:, 2:]
        velocity = np.array([.04, -.02, 2.12])
        times = [0., .8, 1., 1.2, 1.4, 1.6, 1.8] if times is None else times
        gravity = np.array([0., 0., 9.81])
        states, frames, edges = [], [], []
        for i, stamp in enumerate(times):
            position = np.array([0., 0., 2.*stamp])
            states.append(np.r_[velocity*stamp, velocity, np.zeros(3)].tolist())
            frames.append(dict(ids=list(range(16)) if i else [],
                pixels=project(world-position).tolist() if i else [],
                gate_pixels=(project(gate-position)+rng.normal(0, .15, (4, 2))).tolist() if 1. <= stamp <= last_gate_time else []))
            if i:
                dt = stamp-times[i-1]
                edges.append(dict(first=i-1, last=i, dt=dt, dv=(-gravity*dt).tolist(), dp=(-.5*gravity*dt**2).tolist()))
        prior = np.log(world[:, 2]*1.06)
        request = dict(model_version=2, fixed_anchor_position=True, anchor_velocity_sigma=.05,
            intrinsics=k.tolist(), rays=(world/world[:, 2:]).tolist(), depth_prior=prior.tolist(),
            depth_sigma=np.full(16, .5).tolist(), states=states, frames=frames, edges=edges,
            gravity=gravity.tolist(), anchor=states[0][:6], depths=prior.tolist(), gate_world=gate.tolist())
        return request,world,times

    def test_returning_square_corrects_velocity_and_depth_scale(self):
        request,world,times = self.problem()
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

    def test_retained_gate_views_constrain_motion_after_dropout(self):
        request,_,times = self.problem([0.,.8,1.,1.2,2.6,2.8,3.],last_gate_time=1.2)
        corrected = solve_window(request)
        forgotten = copy.deepcopy(request)
        for frame in forgotten['frames']:
            frame['gate_pixels'] = []
        off = solve_window(forgotten)
        truth = np.array([0.,0.,2.])
        velocity_error = lambda fit: np.linalg.norm(np.array(fit['states'])[-1,3:6]-truth)
        self.assertLess(velocity_error(corrected),.1)
        self.assertLess(velocity_error(corrected),velocity_error(off)*.5)
        self.assertLess(np.linalg.norm(np.array(corrected['states'])[-1,:3]-truth*times[-1]),.15)
        self.assertGreater(velocity_error(off),.08)


if __name__ == '__main__':
    unittest.main()
