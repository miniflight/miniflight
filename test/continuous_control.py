import unittest
from unittest.mock import patch
import numpy as np
from miniflight.replay import Calls
from miniflight.gate_filter import GateFilter,GateHistory
from target.aigp.experiments.camera_imu import sensors,vision,continuous_control


class ContinuousControlTest(unittest.TestCase):
    def setUp(self):
        settings=patch.dict(sensors.SETTINGS,view_recovery=True);settings.start();self.addCleanup(settings.stop)

    def test_disabled_candidate_preserves_the_normal_controller(self):
        c,f=self.controller();f.last_visual[1]=0.;c.inertial.history.time=.6;c.inertial.sync()
        with patch.dict(sensors.SETTINGS,view_recovery=False):
            c.drive(None);self.assertIsNone(c.recovery_stop);self.assertTrue(c.final_approach_ready())
    def test_final_commit_requires_a_fresh_full_opening_and_no_recovery(self):
        c,f=self.controller();f.last_full_visual.clear();self.assertFalse(c.final_approach_ready())
        f.last_full_visual[1]=0.;self.assertTrue(c.final_approach_ready())
        c.recovery_stop=-6.;self.assertFalse(c.final_approach_ready())

    def test_height_changes_end_with_level_crossing_velocity(self):
        c,f=self.controller();index,center,yaw=f.gates[2];f.gates[2]=(index,center+[0.,0.,8.],yaw)
        c.drive(None)
        self.assertEqual(c.curve.end_direction[2],0.)
        self.assertAlmostEqual(c.curve.geometry(1.)[1][2],0.,places=12)
        c.gate_number=2;c.build(f,f.gates[2][1],index,c.end_direction(f,f.gates[2][1]))
        self.assertAlmostEqual(c.curve.geometry(1.)[0][2],0.,places=12)
        self.assertAlmostEqual(c.curve.geometry(1.)[1][2],0.,places=12)

    def controller(self):
        model=dict(hover_thrust=.27,thrust_acceleration=60.,position_gain=4/3,velocity_gain=3.,attitude_gain=8.,max_rate=3.,
            thrust_curve=(18.,65.),body_resistance_per_m=(.047,)*3)
        with patch.dict(sensors.SETTINGS,position_config=model,speed_m_s=5.):c=continuous_control.GateController(Calls())
        f=GateFilter(np.eye(3),np.zeros(3),model['thrust_curve'],model['body_resistance_per_m'],vision.CAMERA_FROM_BODY,vision.K)
        for key,center in [(1,[20.,0.,-2.]),(2,[45.,3.,-5.])]:f.add_gate(key,vision.CAMERA_FROM_BODY @ center,vision.CAMERA_FROM_BODY @ [0.,1.,0.])
        f.last_full_visual[1]=0.
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
            c.inertial.history.time=float(t);f.last_full_visual[1]=float(t);c.inertial.sync();request=c.drive(None)
            self.assertTrue(np.isfinite([request.roll_rate,request.pitch_rate,request.yaw_rate,request.thrust]).all())
            self.assertGreaterEqual(request.thrust,.21);self.assertLessEqual(request.thrust,.325)
        self.assertGreater(c.signal['reference']['velocity'][0],0.)
        self.assertFalse(c.signal['crossing_alignment']);self.assertFalse(c.signal['visibility_alignment'])

    def test_hover_balances_the_estimated_force_error(self):
        c,f=self.controller();f.ba[2]=.5;c.inertial.sync()
        request=c.drive(None);a,b=f.thrust_curve
        upward=a*request.thrust+b*request.thrust**2
        self.assertAlmostEqual(9.81-upward+f.ba[2],0.,places=10)

    def test_recovery_brakes_before_an_off_height_crossing(self):
        c,f=self.controller();f.p[0]=14.;f.v[0]=5.;f.last_visual[1]=0.;c.inertial.sync()
        c.drive(None)
        self.assertTrue(c.signal['reference']['recovery'])
        np.testing.assert_array_equal(c.signal['reference']['velocity'],[0.,0.,0.])
        self.assertEqual(c.signal['reference']['position_relative_gate'][2],0.)

    def test_far_recovery_approaches_slowly_instead_of_waiting_outside_view(self):
        c,f=self.controller();f.last_visual[1]=0.;c.inertial.history.time=.6;c.inertial.sync();c.drive(None)
        self.assertTrue(c.signal['reference']['recovery'])
        self.assertEqual(c.signal['reference']['velocity'][0],3.)
        self.assertLessEqual(c.recovery_budget_s,8.)

    def test_recovery_releases_after_fresh_aligned_stationary_view(self):
        c,f=self.controller();f.last_visual[1]=0.;c.inertial.history.time=.6;c.inertial.sync();c.drive(None)
        self.assertIsNotNone(c.recovery_stop)
        f.p[2]=-2.;f.v[:]=0.;f.last_visual[1]=.6;f.last_full_visual[1]=.6;c.inertial.sync();c.drive(None)
        self.assertIsNone(c.recovery_stop);self.assertIsNone(c.curve)

    def test_recovery_release_does_not_chatter_on_small_alignment_changes(self):
        c,f=self.controller();f.last_visual[1]=0.;c.inertial.history.time=.6;c.inertial.sync();c.drive(None)
        f.p[2]=-2.;f.v[0]=1.2;f.last_visual[1]=.6;f.last_full_visual[1]=.6;c.inertial.sync();c.drive(None)
        self.assertIsNone(c.recovery_stop)
        f.p[0]=14.;f.p[2]=-1.4;c.inertial.history.time=.62;f.last_visual[1]=.62;c.inertial.sync();c.drive(None)
        self.assertIsNone(c.recovery_stop)

    def test_rejected_lookahead_cannot_be_promoted_to_a_lock(self):
        c,f=self.controller();c.gate_number=2
        self.assertFalse(c.lookahead_admitted(dict(time=1.)))
        f.last_visual[2]=.9
        self.assertFalse(c.lookahead_admitted(dict(time=1.)))
        self.assertTrue(c.lookahead_admitted(dict(time=.9)))

    def test_known_map_guides_acquisition_without_claiming_a_visual_lock(self):
        c,f=self.controller();c.phase=sensors.Phase.ACQUIRING;c.locked=None;c.direction=None
        with patch.dict(sensors.SETTINGS,map_prior=f.map_state()):
            for t in np.arange(0.,.5,.02):
                c.inertial.history.time=float(t);c.inertial.sync();c.drive(None)
        self.assertGreater(c.signal['reference']['velocity'][0],0.);self.assertIsNone(c.locked)
        self.assertFalse(f.last_visual)

    def test_fresh_bar_cannot_release_recovery_without_full_opening(self):
        c,f=self.controller();c.inertial.history.time=.6;f.last_visual[1]=.6;c.inertial.sync();c.drive(None)
        self.assertIsNotNone(c.recovery_stop)

    def test_upward_camera_moves_commit_before_the_fixed_close_range(self):
        c,f=self.controller();c.drive(None);self.assertGreater(c.final_distance_m,2.5)

    def test_committed_imu_approach_does_not_reenter_visual_hold(self):
        c,f=self.controller();c.phase=sensors.Phase.FINAL_IMU;c.inertial.history.time=.6;c.inertial.sync();c.drive(None)
        self.assertIsNone(c.recovery_stop)

    def test_historical_measurement_cannot_keep_rejected_track_locked(self):
        from types import SimpleNamespace
        from target.aigp.experiments.camera_imu import course
        c,f=self.controller();f.localization=True;f.last_visual[1]=0.;c.inertial.history.time=10.;c.inertial.sync()
        frame=SimpleNamespace(id=10)
        with patch.object(course.GateController,'observe',return_value=None):
            c.observe(frame,9.9,np.zeros(3),np.eye(3))
            self.assertIsNone(c.locked);self.assertEqual(c.acquire_started,10.)
            c.locked=1;c.phase=sensors.Phase.TRACKING;c.inertial.history.time=10.1;c.inertial.sync()
            c.observe(frame,10.,np.zeros(3),np.eye(3))
            self.assertEqual(c.acquire_started,10.)
