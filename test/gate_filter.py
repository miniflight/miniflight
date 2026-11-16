import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from miniflight.gate_filter import GateFilter,GateHistory
from target.aigp.experiments.camera_imu.vision import CAMERA_FROM_BODY,K

POINTS=.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])


def filter():
    f=GateFilter(Rotation.from_euler('xyz',[.05,-.1,.1]).as_matrix(),[.001,-.002,.003],
        [7.7911544524,106.6595510654],[.0476857765]*3,CAMERA_FROM_BODY,K)
    f.p=np.array([.2,-.3,.1]);f.v=np.array([3.,.4,-.2])
    f.add_gate(1,np.array([.2,-1.,15.]),np.array([1.,0.,0.]))
    return f


class JointGateTest(unittest.TestCase):
    def test_full_rim_factor_contains_measured_outer_corners_of_the_same_gate(self):
        import cv2
        from target.aigp.experiments.camera_imu.gate_filter_live import full_rim_observation
        from target.aigp.experiments.camera_imu.vision import best_pose
        def pixels(points):
            camera=(np.array(points)+[12.,0.,-.4]) @ CAMERA_FROM_BODY.T;h=camera @ K.T
            return h[:,:2]/h[:,2,None]
        aperture=pixels(POINTS);outer=pixels([[0.,-1.4,-1.35],[0.,1.4,-1.35],[0.,1.4,1.35],[0.,-1.4,1.35]])
        image=np.zeros((360,640,3),np.uint8);cv2.fillConvexPoly(image,np.rint(outer).astype(np.int32),(0,0,220));cv2.fillConvexPoly(image,np.rint(aperture).astype(np.int32),(0,0,0))
        measured=full_rim_observation(image,aperture,best_pose(aperture));self.assertIsNotNone(measured)
        np.testing.assert_allclose(measured[0],outer[:2],atol=1.3)
        np.testing.assert_allclose([measured[1]['width_m'],measured[1]['center_to_top_m']],[2.8,1.35],atol=.08)
        from unittest.mock import patch
        bad=measured[0].copy();bad[0,0]-=35.
        with patch('target.aigp.experiments.camera_imu.gate_filter_live.outer_top',return_value=(bad,.1)),patch('target.aigp.experiments.camera_imu.gate_filter_live.same_red_component',return_value=True):
            self.assertIsNone(full_rim_observation(image,aperture,best_pose(aperture)))

    def test_partial_pixels_cannot_replace_a_metric_landmark_in_joint_rim_mode(self):
        from unittest.mock import patch
        f=filter();f.v[:]=0.;f.partial_gate_births=False;f.add_rim(2.8,1.35,.1)
        aperture=f.measurement(1,POINTS)[0]
        for _ in range(8):f.observe(1,POINTS,aperture)
        near=f.copy();i,g,yaw=near.gates[1];near.gates[1]=(i,near.p+.4*(g-near.p),yaw)
        pixels=near.measurement(1,None,rim=True)[0];h=GateHistory(f,0.,[0.,0.,0.]);h.imu(.1,[0.,0.,0.])
        with patch.object(GateFilter,'consider_nearer',side_effect=AssertionError('partial rebound')):
            for k in range(3):h.observe(dict(id=k,image_id=k,time=.02+.02*k,key=1,pixels=pixels,points=None,seed=None,rim=True,rim_prior=(2.8,1.35,.1)))
        self.assertEqual(h.filter.retargets,0);self.assertTrue(all(k>0 for k in h.filter.gates))

    def test_joint_aperture_and_rim_measurements_learn_shape_from_broad_seeds(self):
        for scale in [.7,1.3]:
            f=filter();f.attitude_corrections=False;f.broad_rim=True
            index,center,yaw=f.gates[1];gate=Rotation.from_euler('z',yaw).as_matrix()
            points=np.array([[0.,-1.4,-1.35],[0.,1.4,-1.35]])
            camera=((points @ gate.T+center-f.p) @ f.R) @ f.C.T;h=camera @ f.K.T;top=h[:,:2]/h[:,2,None]
            aperture=f.measurement(1,POINTS)[0];f.add_rim(2.8*scale,1.35*scale,.001)
            self.assertGreater(f.P[f.rim[0],f.rim[0]],1.)
            for _ in range(30):f.observe(1,POINTS,aperture);f.observe(1,None,top,rim=True)
            np.testing.assert_allclose(f.rim[1],[2.8,1.35],atol=.015)
            self.assertGreater(np.linalg.norm(f.P[index:index+3,f.rim[0]:f.rim[0]+2]),0.)

    def test_consider_attitude_keeps_its_mean_and_measurement_uncertainty(self):
        f=filter();f.attitude_corrections=False;R=f.R.copy();P=f.P[6:9,6:9].copy()
        pixels=f.measurement(1,POINTS)[0]
        for _ in range(4):
            f.observe(1,POINTS,pixels+[.4,-.3]);expected,_=f.acceleration_measurement([.28]*4)
            f.observe_acceleration(expected+[.2,-.1,.1],[.28]*4,.5)
        np.testing.assert_array_equal(f.R,R);np.testing.assert_allclose(f.P[6:9,6:9],P,atol=1e-15)
        self.assertGreater(np.linalg.norm(f.P[:6,6:9]),0.)

    def test_extended_visual_budget_requires_a_far_known_precise_height_maneuver(self):
        from target.aigp.experiments.camera_imu.gate_filter_live import outage_budget
        r=np.array([30.,1.,10.]);d=np.array([1.,0.,0.]);P=np.eye(3)*.1**2
        budget=outage_budget(r,P,d,True);self.assertEqual(budget['budget_s'],4.5)
        self.assertEqual(outage_budget(r,P,d,False)['budget_s'],2.)
        self.assertEqual(outage_budget([6.,1.,1.],P,d,True)['budget_s'],2.)
        self.assertEqual(outage_budget(r,np.eye(3)*5.**2,d,True)['budget_s'],2.)
        Q=Rotation.from_euler('z',.7).as_matrix();rotated=outage_budget(Q @ r,Q @ P @ Q.T,Q @ d,True)
        self.assertAlmostEqual(rotated['bearing_3sigma_rad'],budget['bearing_3sigma_rad'])

    def test_pad_support_is_not_fitted_as_a_force_bias(self):
        f=GateFilter(np.eye(3),np.zeros(3),[11.81,0.],[0.]*3,CAMERA_FROM_BODY,K,supported=True)
        h=GateHistory(f,0.,[0.,0.,0.]);h.command(.001,[1.]*4)
        for t in [.02,.04,.06]:h.imu(t,[0.,0.,0.],[0.,0.,-9.81])
        self.assertFalse(h.filter.airborne);self.assertEqual(h.filter.force_accepted,0)
        np.testing.assert_array_equal(h.filter.p,np.zeros(3));np.testing.assert_array_equal(h.filter.ba,np.zeros(3))
        h.imu(.08,[0.,0.,0.],[0.,0.,-11.81]);self.assertTrue(h.filter.airborne)
        h.imu(.1,[0.,0.,0.],[0.,0.,-11.81])
        self.assertLess(h.filter.v[2],0.);np.testing.assert_allclose(h.filter.ba,np.zeros(3),atol=1e-12)

    def test_sent_command_does_not_act_before_its_public_epoch(self):
        f=GateFilter(np.eye(3),np.zeros(3),[11.81,0.],[0.]*3,CAMERA_FROM_BODY,K)
        h=GateHistory(f,0.,[0.,0.,0.]);h.command(.025,[1.]*4)
        np.testing.assert_array_equal(h.filter.v,[0.,0.,0.])
        h.imu(.1,[0.,0.,0.])
        np.testing.assert_allclose(h.filter.v,[0.,0.,-.15],atol=1e-12)
        np.testing.assert_allclose(h.filter.p,[0.,0.,-.075**2],atol=1e-12)

    def test_launch_lifts_before_commanding_a_large_rotation(self):
        import json
        from types import SimpleNamespace
        from unittest.mock import patch
        from common.math import Quaternion
        from miniflight.position import PositionConfig
        from target.aigp.experiments.camera_imu import gate_filter_live,course,sensors
        c=object.__new__(gate_filter_live.GateController);c.phase=sensors.Phase.SHIFTING;c.launch_cleared=False;c.signal={}
        c.config=PositionConfig(.27,60.,thrust_curve=(8.,106.));c.inertial=SimpleNamespace(position=np.array([0.,0.,-.1]),rotation=Quaternion.from_euler(0.,-.3,0.))
        request=c.drive(np.array([0.,-.5,-1.]));self.assertEqual((request.roll_rate,request.pitch_rate,request.yaw_rate),(0.,0.,0.));self.assertGreater(request.thrust,.27)
        json.dumps(c.signal)
        c.inertial.position[2]=-.25
        c.inertial.history=SimpleNamespace(filter=SimpleNamespace(airborne=False))
        request=c.drive(np.array([0.,-.5,-1.]));self.assertEqual((request.roll_rate,request.pitch_rate,request.yaw_rate),(0.,0.,0.));self.assertFalse(c.launch_cleared)
        c.inertial.history.filter.airborne=True
        with patch.object(course.GateController,'drive',return_value=None):c.drive(np.array([0.,-.5,-1.]))
        self.assertTrue(c.launch_cleared)
        json.dumps(c.signal)

    def test_upright_seed_recovers_depth_without_using_planar_pitch(self):
        from miniflight.gate_filter import upright_seed
        for yaw in [-.4,0.,.3]:
            R=Rotation.from_euler('xyz',[.07,-.13,.2]).as_matrix();camera=CAMERA_FROM_BODY @ R.T
            center=np.array([15.,1.,-.8]);gate=Rotation.from_euler('z',yaw).as_matrix()
            xyz=(POINTS @ gate.T+center) @ camera.T;h=xyz @ K.T;pixels=h[:,:2]/h[:,2,None]
            translation,right=upright_seed(pixels,camera @ [0.,0.,1.],K)
            np.testing.assert_allclose(translation,camera @ center,atol=1e-10)
            np.testing.assert_allclose(right,camera @ gate @ [0.,1.,0.],atol=1e-10)

    def test_moving_start_preserves_one_frame_and_nonzero_velocity(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from common.math import Quaternion
        from target.aigp.experiments.camera_imu import sensors
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=1.)
        settings=dict(position_config=dict(hover_thrust=.27,thrust_acceleration=60.),visual_initialization=True,continuous_origin=True,moving_start=True)
        with patch.dict(sensors.SETTINGS,**settings),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            c=sensors.GateController();c.phase=sensors.Phase.SHIFTING;c.locked=1;c.setup_started=0.;c.setup_steady=.5
            c.inertial.rotation=Quaternion();c.inertial.time=1.;c.inertial.position=np.array([.2,-.5,-1.]);c.inertial.velocity=np.array([.1,.3,.2])
            c.inertial.times=[.5,1.];c.inertial.rotations=[CAMERA_FROM_BODY.T.copy()]*2
            c.anchor=np.array([20.,1.,2.]);c.last_image_time=1.;c.last_image_id=1;c.direction=np.array([1.,0.,0.])
            c.last_relative_gate=c.anchor-c.inertial.position;c.last_relative_position=c.inertial.position.copy()
            c.last_corners=np.zeros((4,2));c.last_gate_corners=np.zeros((4,3));c.previous=(c.inertial.position.copy(),CAMERA_FROM_BODY.T,np.zeros((4,2)),None)
            with patch.object(c,'drive',return_value=None):c.update((),(),None)
            self.assertIs(c.phase,sensors.Phase.TRACKING);np.testing.assert_array_equal(c.inertial.position,[.2,-.5,-1.])
            np.testing.assert_array_equal(c.inertial.velocity,[.1,.3,.2]);self.assertEqual(c.inertial.times,[.5,1.])

    def test_direct_launch_requires_public_liftoff_and_preserves_flight_velocity(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from common.math import Quaternion
        from target.aigp.experiments.camera_imu import sensors
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=1.)
        settings=dict(position_config=dict(hover_thrust=.27,thrust_acceleration=60.),visual_initialization=True,continuous_origin=True,moving_start=True,direct_launch=True,shift_right_m=.5)
        with patch.dict(sensors.SETTINGS,**settings),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            c=sensors.GateController();c.phase=sensors.Phase.SHIFTING;c.locked=1;c.setup_started=0.;c.setup_steady=.8
            c.inertial.rotation=Quaternion();c.inertial.time=1.;c.inertial.position=np.array([.2,.4,-.5]);c.inertial.velocity=np.array([.7,.4,-.5])
            c.inertial.history=SimpleNamespace(filter=SimpleNamespace(airborne=False));c.inertial.times=[.5,1.];c.inertial.rotations=[CAMERA_FROM_BODY.T.copy()]*2
            c.anchor=np.array([20.,1.,-1.]);c.last_image_time=1.;c.last_image_id=1;c.direction=np.array([1.,0.,0.])
            c.last_relative_gate=c.anchor-c.inertial.position;c.last_relative_position=c.inertial.position.copy()
            c.last_corners=np.zeros((4,2));c.last_gate_corners=np.zeros((4,3));c.previous=(c.inertial.position.copy(),CAMERA_FROM_BODY.T,np.zeros((4,2)),None)
            with patch.object(c,'drive',return_value=None):
                c.update((),(),None);self.assertIs(c.phase,sensors.Phase.SHIFTING)
                c.inertial.history.filter.airborne=True;c.setup_steady=.8;c.update((),(),None)
            self.assertIs(c.phase,sensors.Phase.TRACKING);np.testing.assert_array_equal(c.inertial.position,[.2,.4,-.5]);np.testing.assert_array_equal(c.inertial.velocity,[.7,.4,-.5])

    def test_visual_updates_cannot_turn_real_rotation_into_gyro_bias(self):
        f=filter();prior=f.bg.copy();P=f.P[9:12,9:12].copy();pixels=f.measurement(1,POINTS)[0]
        for _ in range(6):f.observe(1,POINTS,pixels+[.4,-.2])
        np.testing.assert_array_equal(f.bg,prior);np.testing.assert_allclose(f.P[9:12,9:12],P,atol=1e-15)

    def test_joint_anchor_always_reads_the_current_posterior(self):
        from types import SimpleNamespace
        from target.aigp.experiments.camera_imu.gate_filter_live import GateController
        f=filter();c=object.__new__(GateController);c.gate_number=1;c.inertial=SimpleNamespace(history=GateHistory(f,0.,[0.,0.,0.]))
        c.anchor=np.array([999.]*3)
        np.testing.assert_array_equal(c.anchor,f.gates[1][1])
        index=f.gates[1][0];step=np.zeros(len(f.P));step[index]=.7;f.inject(step)
        np.testing.assert_array_equal(c.anchor,f.gates[1][1])
        copied=c.anchor;copied[:]=0.;self.assertGreater(np.linalg.norm(c.anchor),1.)

    def test_rejected_pixels_do_not_refresh_the_joint_measurement_clock(self):
        f=filter();history=GateHistory(f,0.,[0.,0.,0.]);history.imu(.1,[0.,0.,0.])
        pixels=f.measurement(1,POINTS)[0]
        history.observe(dict(id=1,time=.05,key=1,points=POINTS,pixels=pixels))
        self.assertEqual(history.filter.last_visual[1],.05)
        history.observe(dict(id=2,time=.08,key=1,points=POINTS,pixels=pixels+10000))
        self.assertFalse(history.results[2]['accepted']);self.assertEqual(history.filter.last_visual[1],.05)

    def test_recorded_braking_failure_improves_all_four_quantities(self):
        import contextlib,io,tempfile
        from pathlib import Path
        from target.aigp.experiments.camera_imu.gate_filter_replay import run
        from target.aigp.experiments.camera_imu.gate_filter_score import score
        source=Path(__file__).parent/'fixtures/joint_braking'
        model=Path(__file__).parents[1]/'target/aigp/experiments/camera_imu/dynamics.model.json'
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()):
            output=Path(directory)/'replay';d=run(source,output,model,source/'public-tracks.jsonl.gz');s=score(source,output,render=False)
            for quantity in ['position','velocity','attitude','depth']:
                self.assertLess(s['braking']['joint'][quantity]['rms'],s['braking']['baseline'][quantity]['rms'],quantity)
            self.assertGreater(d['minimum_covariance_eigenvalue'],-1e-9)
            self.assertEqual(d['late_images_rejected'],0)
            with np.load(output/'estimates.npz',allow_pickle=False) as data:
                self.assertGreater(data['gate_available'].sum(),0)
                self.assertTrue(np.isfinite(data['camera_gate'][data['gate_available']]).all())

    def test_settling_completion_wins_on_the_deadline_tick(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from common.math import Quaternion
        from target.aigp.experiments.camera_imu import sensors
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=10.03)
        with patch.dict(sensors.SETTINGS,position_config=dict(hover_thrust=.27,thrust_acceleration=60.),visual_initialization=True),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            c=sensors.GateController();c.phase=sensors.Phase.SHIFTING;c.locked=1;c.setup_started=0.;c.setup_steady=9.62
            c.inertial.rotation=Quaternion();c.inertial.time=10.03;c.inertial.position=np.array([0.,-.5,-1.]);c.inertial.velocity=np.array([.1,.08,.04])
            c.anchor=np.array([20.,1.,2.]);c.direction=np.array([1.,0.,0.]);c.last_image_time=10.03;c.last_image_id=1
            c.last_relative_position=c.inertial.position.copy();c.last_relative_gate=c.anchor-c.inertial.position
            c.last_corners=np.array([[200.,150.],[280.,150.],[280.,230.],[200.,230.]])
            c.previous=(c.inertial.position.copy(),CAMERA_FROM_BODY.T,np.zeros((4,2)),None);c.last_gate_corners=np.zeros((4,3))
            with patch.object(c,'drive',return_value=None):c.update((),(),None)
            self.assertIs(c.phase,sensors.Phase.TRACKING);np.testing.assert_array_equal(c.inertial.velocity,[.1,.08,.04])

    def test_known_size_cannot_absorb_motion_or_projection_error(self):
        f=filter();expected,_=f.measurement(1,POINTS);variance=f.P[15,15]
        for _ in range(6):f.observe(1,POINTS,expected*1.01)
        self.assertEqual(f.size,1.5);self.assertAlmostEqual(f.P[15,15],variance,places=14)

    def test_joint_one_gate_feedback_cannot_run_blind_for_thirty_seconds(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from target.aigp.experiments.camera_imu import gate_filter_live,course,sensors
        c=object.__new__(gate_filter_live.GateController);c.locked=0;c.phase=sensors.Phase.TRACKING;c.signal={}
        c.inertial=SimpleNamespace(time=5.);c.measurement_time=lambda:2.
        with patch.object(course.GateController,'update',return_value=None):
            with self.assertRaisesRegex(TimeoutError,'two seconds'):c.update((),(),None)

    def test_accelerometer_factor_updates_force_bias_without_interval_assumption(self):
        f=filter();u=np.full(4,.27);expected,H=f.acceleration_measurement(u);eps=1e-6;numeric=[]
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.acceleration_measurement(u)[0]-b.acceleration_measurement(u)[0])/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)
        before=np.linalg.norm(expected-(expected+[0.,0.,-.3]))
        f.observe_acceleration(expected+[0.,0.,-.3],u,.5)
        self.assertLess(np.linalg.norm(f.acceleration_measurement(u)[0]-(expected+[0.,0.,-.3])),before)

    def test_live_origin_shift_preserves_joint_relative_geometry_and_covariance(self):
        from miniflight.position import PositionConfig
        from target.aigp.experiments.camera_imu.gate_filter_live import Inertial
        f=filter();observer=Inertial(PositionConfig(.27,60.,thrust_curve=(8.,106.),body_resistance_per_m=(.05,)*3))
        observer.history=GateHistory(f,1.,[0.,0.,0.]);observer.times=[1.];observer.positions=[f.p.copy()]
        observer.velocities=[f.v.copy()];observer.rotations=[f.R @ CAMERA_FROM_BODY.T]
        before=f.relative(1)[0];cov=f.P.copy();pixels=f.measurement(1,POINTS)[0]
        observer.reset_origin(np.array([2.,-.5,1.]))
        np.testing.assert_allclose(observer.history.filter.relative(1)[0],before)
        np.testing.assert_array_equal(observer.history.filter.P,cov)
        np.testing.assert_allclose(observer.history.filter.measurement(1,POINTS)[0],pixels)

    def test_nearer_binding_needs_three_distinct_images_and_keeps_old_landmark(self):
        f=filter();f.gates[1]=(f.gates[1][0],f.p+np.array([40.,0.,0.]),0.)
        camera=f.C @ f.R.T;seed=(camera @ [12.,0.,0.],camera @ [0.,1.,0.]);before=f.p.copy()
        for _ in range(3):self.assertFalse(f.consider_nearer(1,7,1.,seed))
        self.assertFalse(f.consider_nearer(1,8,1.03,seed));self.assertTrue(f.consider_nearer(1,9,1.06,seed))
        np.testing.assert_allclose(f.gates[1][1]-f.p,[12.,0.,0.]);np.testing.assert_array_equal(f.p,before)
        self.assertEqual(len(f.gates),2);self.assertEqual(f.retargets,1)

    def test_track_export_is_independent_of_previous_motion_estimates(self):
        import gzip,json,tempfile
        from pathlib import Path
        from target.aigp.experiments.camera_imu.gate_filter_inputs import export
        row=dict(gate_number=1,motor_commands=[.27]*4,position=[100]*3,velocity=[100]*3,camera_orientation=np.eye(3).tolist(),
            camera_observation=dict(image_id=7,capture_time=1.,corners=[[1,2],[3,2],[3,4],[1,4]],reprojection_rmse_px=.2))
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);np.savez(root/'public-inputs.npz',motor_sent_host_time=[1001.],motor_outputs=[[.27]*4])
            def write():
                with gzip.open(root/'control.jsonl.gz','wt') as stream:stream.write(json.dumps(row)+'\n')
            write();export(root,root/'first.gz');row.update(position=[-999]*3,velocity=[-999]*3,camera_orientation=(-np.eye(3)).tolist())
            write();export(root,root/'second.gz')
            with gzip.open(root/'first.gz','rt') as a,gzip.open(root/'second.gz','rt') as b:self.assertEqual(a.read(),b.read())

    def test_partial_pixels_jointly_estimate_uncertain_rim_dimensions(self):
        f=filter();f.add_rim(2.84,1.4,.1);expected,H=f.measurement(1,None,rim=True);eps=1e-6
        numeric=[]
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.measurement(1,None,True)[0]-b.measurement(1,None,True)[0]).ravel()/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)
        before=f.P.copy();result=f.observe(1,None,expected+[.1,-.2],rim=True)
        self.assertTrue(result['accepted']);self.assertLess(np.trace(f.P),np.trace(before))
        index=f.gates[1][0];rim=f.rim[0]
        self.assertGreater(np.linalg.norm(f.P[index:index+4,rim:rim+2]),1e-8)

    def test_raw_pixel_jacobian_includes_attitude_and_gate_depth(self):
        f=filter();pixels,H=f.measurement(1,POINTS);numeric=[];eps=1e-6
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.measurement(1,POINTS)[0]-b.measurement(1,POINTS)[0]).ravel()/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)

    def test_process_jacobian_repropagates_corrected_attitude_into_motion(self):
        before=filter();nominal=before.copy();gyro=np.array([.2,-.3,.1]);u=np.array([.26,.28,.27,.25])
        F=nominal.predict(.013,gyro,u);numeric=[];eps=1e-6
        def difference(a,b):
            tail=np.concatenate([np.r_[a.gates[k][1]-b.gates[k][1],a.gates[k][2]-b.gates[k][2]] for k in a.gates])
            return np.r_[a.p-b.p,a.v-b.v,Rotation.from_matrix(b.R.T @ a.R).as_rotvec(),a.bg-b.bg,a.ba-b.ba,a.size-b.size,tail]
        for column in range(len(F)):
            a=before.copy();b=before.copy();step=np.eye(len(F))[column]*eps;a.inject(step);b.inject(-step)
            a.predict(.013,gyro,u);b.predict(.013,gyro,u)
            numeric.append((difference(a,nominal)-difference(b,nominal))/(2*eps))
        np.testing.assert_allclose(F,np.array(numeric).T,atol=2e-7)

    def test_late_pixels_equal_chronological_joint_update_after_braking(self):
        f=filter();a=GateHistory(f.copy(),0.,[0.,0.,0.]);b=GateHistory(f.copy(),0.,[0.,0.,0.])
        u=np.full(4,.2689415);a.command(.001,u);b.command(.001,u)
        rows=[]
        for stamp in [.2,.4,.6]:
            truth=f.copy();truth.p+=truth.v*stamp;truth.R=truth.R @ Rotation.from_euler('y',-.1*stamp).as_matrix()
            rows.append(dict(id=len(rows),time=stamp,key=1,points=POINTS,pixels=truth.measurement(1,POINTS)[0]))
        for i in range(1,101):
            stamp=i*.01;gyro=[0.,-.1 if stamp<.5 else .1,0.]
            a.imu(stamp,gyro,[0.,0.,-9.81]);b.imu(stamp,gyro,[0.,0.,-9.81])
            for row in rows:
                if abs(stamp-row['time'])<1e-9:a.observe(row)
                if abs(stamp-row['time']-.13)<1e-9:b.observe(row)
        np.testing.assert_allclose(a.filter.p,b.filter.p,atol=1e-10)
        np.testing.assert_allclose(a.filter.v,b.filter.v,atol=1e-10)
        np.testing.assert_allclose(a.filter.R,b.filter.R,atol=1e-10)
        np.testing.assert_allclose(a.filter.P,b.filter.P,atol=1e-10)
        self.assertEqual(a.rejected_late,0);self.assertEqual(b.rejected_late,0)
        self.assertGreater(np.linalg.eigvalsh(a.filter.P).min(),-1e-10)
        self.assertGreater(np.linalg.norm(a.filter.P[:6,6:9]),1e-5)


if __name__=='__main__':unittest.main()
