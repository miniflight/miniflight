import unittest
from types import SimpleNamespace
import numpy as np
from target.aigp.experiments.camera_imu.camera_gaps import CameraGaps
from target.aigp.experiments.camera_imu.proprioception import predict


class CameraGapTest(unittest.TestCase):
    def test_only_camera_frames_are_withheld_and_late_gap_frames_stay_withheld(self):
        recorded=[];mask=CameraGaps(.2,lambda **r:recorded.append(r))
        frame=lambda stamp:SimpleNamespace(id=round(stamp*1000),time_ns=round(stamp*1e9))
        self.assertEqual(len(mask.select((frame(1.),),1.,False)),1)
        mask.select((),2.,True)
        selected=mask.select(tuple(map(frame,[2.99,3.,3.19,3.21,6.1,6.21])),7.,False)
        self.assertEqual([f.id for f in selected],[2990,3210,6210])
        self.assertEqual([r['image_id'] for r in recorded],[3000,3190,6100])

    def test_invalid_gap_is_rejected(self):
        for duration in [-1.,3.,float('nan')]:
            with self.assertRaises(ValueError):CameraGaps(duration)


class ResponseTest(unittest.TestCase):
    def test_forecast_cannot_read_future_commands_gyro_or_force(self):
        d=dict(time=np.array([0.,.1,.2]),omega=np.zeros((3,3)),force=np.tile([0.,0.,-9.81],(3,1)),commands=np.array([-.1,.02]),motor_outputs=np.full((2,4),.3))
        state=dict(time=0.,rotation=np.eye(3).tolist(),velocity=[0.,0.,0.],gyro_bias=[0.]*3,force_bias=[0.]*3)
        model=dict(position_config=dict(thrust_curve=[32.7,0.],body_resistance_per_m=[0.]*3),rotation_coefficients=[[1.,1.,0.]]*3,force_lag_s=.01,rotation_lag_s=.01)
        a=predict(d,[state],model,horizons=(.1,))
        d['motor_outputs'][1]=[.9,.1,.2,.7];d['omega'][1:]=100.;d['force'][1:]=100.
        b=predict(d,[state],model,horizons=(.1,))
        for key in a:np.testing.assert_array_equal(a[key],b[key])
        np.testing.assert_allclose(a['force'],[[0.,0.,-9.81]],atol=1e-12)
        np.testing.assert_allclose(a['delta'],[np.eye(3)],atol=1e-12)
