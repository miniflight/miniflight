import json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from target.aigp.experiments.camera_imu.route_prior import load,projected
from test.continuous_control import ContinuousControlTest


class RoutePriorTest(unittest.TestCase):
    def test_prior_is_rebased_on_current_observed_gates(self):
        prior=dict(points={'1':[20.,0.,0.],'2':[45.,3.,5.],'3':[70.,-1.,13.]})
        before=np.array([120.,30.,5.]);center=np.array([145.,33.,10.]);result=projected(prior,2,3,before,center)
        Q=Rotation.from_euler('z',.4);shift=np.array([7.,8.,9.]);changed=dict(points={k:(Q.apply(v)+shift).tolist() for k,v in prior['points'].items()})
        np.testing.assert_allclose(projected(changed,2,3,before,center),result,atol=1e-12)

    def test_planning_prior_does_not_initialize_navigation_landmarks(self):
        c,f=ContinuousControlTest().controller();f.gates.pop(2);size=len(f.P)
        c.route_prior=dict(points={'1':[20.,0.,-2.],'2':[45.,3.,-5.],'3':[70.,-1.,-12.]})
        c.drive(None)
        self.assertNotIn(2,f.gates);self.assertEqual(len(f.P),size)
        self.assertIsNotNone(c.entry_speed);self.assertEqual(c.signal['reference']['route_prior_roles'],[2,3])

    def test_route_schema_rejects_native_fields(self):
        gate=dict(id=1,center=[20.,0.,-1.],yaw=0.,covariance=np.eye(4).tolist(),last_seen=1.)
        data=dict(frame='Initial stationary camera/IMU local frame',source='Public pixels, IMU and own commands only',gates=[gate])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'map.json';path.write_text(json.dumps(data));self.assertEqual(load(path)['points']['1'],[20.,0.,-1.])
            gate['native_position']=[1234.,0.,0.];path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'Unexpected'):load(path)
