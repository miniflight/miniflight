import unittest
import numpy as np
from miniflight.gate_path import GateCurve,SpeedReference,unit,crossing_direction


class GatePathTest(unittest.TestCase):
    def test_crossings_carry_only_consistent_height_changes(self):
        previous=np.array([0.,0.,0.]);center=np.array([24.,0.,5.]);following=np.array([50.,1.,13.])
        direction=crossing_direction(previous,center,following)
        slope=direction[2]/np.linalg.norm(direction[:2]);self.assertGreater(slope,5/24);self.assertLess(slope,8/np.linalg.norm([26.,1.]))
        self.assertEqual(crossing_direction(previous,center,[50.,0.,4.])[2],0.)
        self.assertEqual(crossing_direction(previous,center,[50.,0.,5.])[2],0.)
        self.assertEqual(crossing_direction(previous,center)[2],0.)
        curve=GateCurve(previous-center,[1.,0.,0.],direction)
        self.assertGreaterEqual(min(curve.derivatives(u)[1][2] for u in np.linspace(0.,1.,101)),-1e-10)

    def test_short_replans_cannot_fold_backward(self):
        c=GateCurve([-.1,0.,0.],[1.,0.,0.],[1.,0.,0.])
        self.assertTrue(all(c.derivatives(u)[1][0]>0. for u in np.linspace(0.,1.,101)))
        with self.assertRaises(ValueError):GateCurve([0.,0.,0.],[1.,0.,0.],[1.,0.,0.])

    def test_curve_crosses_the_gate_with_nonzero_smooth_tangent(self):
        start=np.array([-24.,2.,-6.]);end=unit([1.,-.1,.3]);curve=GateCurve(start,[1.,0.,0.],end)
        np.testing.assert_allclose(curve.geometry(0.)[0],start)
        np.testing.assert_allclose(curve.geometry(1.)[0],[0.,0.,0.],atol=1e-12)
        np.testing.assert_allclose(curve.geometry(1.)[1],end,atol=1e-12)
        np.testing.assert_allclose(curve.geometry(1.)[2],[0.,0.,0.],atol=1e-12)
        for u in np.linspace(0.,1.,25):
            p=curve.geometry(u)[0];self.assertAlmostEqual(curve.closest(p),u,places=6)

    def test_replanning_preserves_the_current_reference_curvature(self):
        curve=GateCurve([-20.,0.,-8.],[1.,0.,0.],unit([1.,.2,.2]));p,t,k,_=curve.geometry(.4)
        revised=GateCurve(p,t,[1.,0.,0.],k)
        for actual,wanted in zip(revised.geometry(0.)[:3],[p,t,k]):np.testing.assert_allclose(actual,wanted,atol=1e-12)

    def test_speed_changes_respect_acceleration_and_jerk_limits(self):
        r=SpeedReference();last_a=0.
        for target in [5.]*250+[8.]*250+[3.]*250:
            speed,a=r.update(target,.02);self.assertLessEqual(abs(a),3.+1e-10);self.assertLessEqual(abs(a-last_a)/.02,6.+1e-8);last_a=a
        self.assertLess(abs(speed-3.),.03)
        self.assertLess(abs(a),.01)

    def test_reference_acceleration_matches_the_spatial_derivative(self):
        c=GateCurve([-30.,0.,-10.],[1.,0.,0.],unit([1.,0.,.2]));u=.4;_,t,k,s=c.geometry(u);speed=5.;dt=1e-5
        next_t=c.geometry(u+speed*dt/s)[1]
        np.testing.assert_allclose((next_t-t)*speed/dt,k*speed*speed,atol=2e-4)
