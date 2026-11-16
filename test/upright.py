import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from miniflight.upright import edge_gravity,split_upright
from test.gate_filter import filter,POINTS


class UprightTest(unittest.TestCase):
    def test_edge_pixel_attitude_and_epoch_derivatives(self):
        f=filter();pixels=f.measurement(1,POINTS)[0];omega=np.array([.2,-.3,.1])
        h,G,H,time=edge_gravity(pixels,f.R,f.C,f.K,omega,0.);np.testing.assert_allclose(h,0.,atol=1e-12)
        for k in range(8):
            p=pixels.copy().reshape(-1);p[k]+=1e-5;new=edge_gravity(p.reshape(4,2),f.R,f.C,f.K,omega,0.)[0]
            np.testing.assert_allclose((new-h)/1e-5,G[:,k],atol=1e-7)
        for k in range(3):
            step=np.eye(3)[k]*1e-7;R=f.R @ Rotation.from_rotvec(step).as_matrix()
            new=edge_gravity(pixels,R,f.C,f.K,omega,0.)[0];np.testing.assert_allclose((new-h)/1e-7,H[:,k],atol=1e-6)
        new=edge_gravity(pixels,f.R,f.C,f.K,omega,1e-6)[0];np.testing.assert_allclose((new-h)/1e-6,time,atol=1e-6)

    def test_gravity_and_metric_rows_partition_the_same_pixel_information(self):
        f=filter();pixels,H=f.measurement(1,POINTS);V=np.diag(np.linspace(.4,1.,8)**2)
        s=split_upright(pixels,f.R,f.C,f.K,V,np.zeros(3));Q=np.vstack([s['noise_projection'],s['remaining']])
        np.testing.assert_allclose(Q @ V @ Q.T,np.eye(8),atol=1e-12)
        gravity=np.zeros((2,len(f.P)));gravity[:,6:9]=s['attitude']
        np.testing.assert_allclose(s['noise_projection'] @ H,gravity,atol=1e-9)
        split=np.vstack([gravity,s['remaining'] @ H])
        np.testing.assert_allclose(split.T @ split,H.T @ np.linalg.solve(V,H),atol=1e-8)

    def test_upright_pixels_correct_roll_while_other_attitude_factors_are_held(self):
        truth=filter();pixels=truth.measurement(1,POINTS)[0];f=truth.copy();f.attitude_corrections=False;f.upright_attitude=True
        f.R=Rotation.from_euler('x',.02).as_matrix() @ f.R;before=Rotation.from_matrix(f.R.T @ truth.R).magnitude()
        for _ in range(30):f.observe(1,POINTS,pixels)
        after=Rotation.from_matrix(f.R.T @ truth.R).magnitude();self.assertLess(after,.4*before);self.assertGreater(f.gravity_accepted,0)

