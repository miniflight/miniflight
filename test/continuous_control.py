import unittest
from unittest.mock import patch
import numpy as np
from miniflight.replay import Calls
from miniflight.gate_filter import GateFilter,GateHistory
from target.aigp.experiments.camera_imu import sensors,vision,continuous_control


class ContinuousControlTest(unittest.TestCase):
    def controller(self):
        model=dict(hover_thrust=.27,thrust_acceleration=60.,position_gain=4/3,velocity_gain=3.,attitude_gain=8.,max_rate=3.,
            thrust_curve=(18.,65.),body_resistance_per_m=(.047,)*3)
        with patch.dict(sensors.SETTINGS,position_config=model,speed_m_s=5.):c=continuous_control.GateController(Calls())
        f=GateFilter(np.eye(3),np.zeros(3),model['thrust_curve'],model['body_resistance_per_m'],vision.CAMERA_FROM_BODY,vision.K)
        for key,center in [(1,[20.,0.,-2.]),(2,[45.,3.,-5.])]:f.add_gate(key,vision.CAMERA_FROM_BODY @ center,vision.CAMERA_FROM_BODY @ [0.,1.,0.])
        c.inertial.history=GateHistory(f,0.,[0.,0.,0.]);c.inertial.sync();c.phase=sensors.Phase.TRACKING;c.locked=1;c.direction=np.array([1.,0.,0.])
        return c,f

    def test_common_body_map_translation_does_not_change_the_motor_request(self):
        c,f=self.controller();c.drive(None);c.inertial.history.time=.02;c.inertial.sync();first=c.drive(None)
        shift=np.array([3.,-2.,1.]);f.p+=shift;f.gates={k:(i,g+shift,yaw) for k,(i,g,yaw) in f.gates.items()};c.inertial.sync()
        second=c.drive(None)
        np.testing.assert_allclose([first.roll_rate,first.pitch_rate,first.yaw_rate,first.thrust],[second.roll_rate,second.pitch_rate,second.yaw_rate,second.thrust],atol=1e-12)

    def test_normal_reference_keeps_forward_motion_without_settle_modes(self):
        c,f=self.controller()
        for t in np.arange(0.,1.,.02):
            c.inertial.history.time=float(t);c.inertial.sync();request=c.drive(None)
            self.assertTrue(np.isfinite([request.roll_rate,request.pitch_rate,request.yaw_rate,request.thrust]).all())
            self.assertGreaterEqual(request.thrust,.21);self.assertLessEqual(request.thrust,.325)
        self.assertGreater(c.signal['reference']['velocity'][0],0.)
        self.assertFalse(c.signal['crossing_alignment']);self.assertFalse(c.signal['visibility_alignment'])
