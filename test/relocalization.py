import gzip,json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from miniflight.gate_filter import GateFilter
from miniflight.relocalization import locate,bootstrap,POINTS
from target.aigp.experiments.camera_imu.vision import K,CAMERA_FROM_BODY as C
from target.aigp.experiments.camera_imu.gate_filter_events import export,replay,state


def scene(repeated=False,extra=False,seconds=2.3):
    f=GateFilter(np.eye(3),np.zeros(3),[18.,65.],[.047]*3,C,K)
    centers=[[20.,0.,-1.],[45.,2.,4.],[75.,-2.,12.],[110.,5.,24.]]
    if repeated:centers=[[20.*i,0.,-1.] for i in range(1,5)]
    for key,center in zip([7,41,3,9],centers):f.add_gate(key,C @ center,C @ [0.,1.,0.])
    f.P[16:,16:]=np.eye(16)*.2**2
    prior=f.map_state();p=np.array([3.,-1.,-.6]);v=np.array([.3,-.1,.05]);R0=Rotation.from_euler('zyx',[.18,-.15,.03]).as_matrix();omega=np.array([0.,0.,.04])
    imu=[];views=[]
    for t in np.arange(0.,seconds+.101,.01):
        R=R0 @ Rotation.from_rotvec(omega*t).as_matrix();imu.append([t,*(R.T @ [0.,0.,-9.81]),*omega])
    for j,t in enumerate(np.linspace(.1,seconds,6)):
        R=R0 @ Rotation.from_rotvec(omega*t).as_matrix();pixels=[]
        for center in centers[:3 if extra else 2]:
            cam=(POINTS+center-p-v*t) @ R @ C.T;h=cam @ K.T;pixels.append((h[:,:2]/h[:,2,None]).tolist())
        views.append(dict(time=float(t),image_id=j,pixels=pixels))
    return prior,views,np.array(imu),p+seconds*v,v,R0 @ Rotation.from_rotvec(omega*seconds).as_matrix()


class RelocalizationTest(unittest.TestCase):
    def test_unknown_pose_velocity_and_gate_identity_are_recovered(self):
        prior,views,imu,p,v,R=scene();result=locate(prior,views,imu,C,K)
        self.assertTrue(result['accepted'],result);self.assertEqual(result['ids'],[7,41])
        np.testing.assert_allclose(result['position'],p,atol=.02);np.testing.assert_allclose(result['velocity'],v,atol=.02)
        self.assertLess(Rotation.from_matrix(np.array(result['rotation']).T @ R).magnitude(),.002)

    def test_isolated_worker_reproduces_the_numerical_pose_exactly(self):
        from target.aigp.experiments.camera_imu.unknown_start import solve
        prior,views,imu,*_=scene();direct=locate(prior,views,imu,C,K);isolated,profile=solve(prior,views,imu)
        self.assertEqual(isolated,direct);self.assertGreater(profile['wall_ns'],0)

    def test_repeated_geometry_stays_ambiguous_instead_of_choosing_the_launch_gate(self):
        prior,views,imu,*_=scene(repeated=True);result=locate(prior,views,imu,C,K)
        self.assertFalse(result['accepted']);self.assertEqual(result['reason'],'ambiguous');self.assertLess(result['margin'],1.)

    def test_third_gate_confirms_a_shorter_window(self):
        prior,views,imu,p,v,R=scene(extra=True,seconds=1.1);result=locate(prior,views,imu,C,K)
        self.assertTrue(result['accepted']);self.assertEqual(set(result['ids']+result['verification_ids']),{7,41,3})

    def test_earlier_third_gate_remains_evidence_after_leaving_view(self):
        prior,views,imu,*_=scene(extra=True,seconds=1.1);views[-1]['pixels']=views[-1]['pixels'][:2];result=locate(prior,views,imu,C,K)
        self.assertTrue(result['accepted']);self.assertEqual(result['verified_gates'],1);self.assertEqual(result['verification_ids'],[])

    def test_cold_bootstrap_replays_mean_covariance_and_inherited_map_correlations(self):
        prior,views,imu,*_=scene();config=dict(thrust_curve=[18.,65.],drag=[.047]*3,camera_from_body=C.tolist(),intrinsics=K.tolist(),
            supported=False,attitude_corrections=False,history_s=2.,map_prior=prior,bootstrap_views=views,image_time_sigma_s=.01)
        history,result=bootstrap(config,imu,views)
        self.assertGreater(np.linalg.norm(history.filter.P[:3,16:]),0.)
        self.assertGreater(np.linalg.norm(history.filter.p),1.)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);raw=root/'flight.jsonl';rows=[dict(event='estimator_input',host_time=0.,sequence=1,kind='init',seed=imu.tolist(),config=config),dict(event='estimator_state',host_time=0.,sequence=1,state=state(history))]
            raw.write_text(''.join(json.dumps(r)+'\n' for r in rows));export(raw,root);report=replay(root,root/'exact')
            self.assertTrue(report['exact_mean_and_covariance']);self.assertEqual(report['matched_live_states'],1)

    def test_private_pose_cannot_enter_localization_observations(self):
        prior,views,imu,*_=scene();views[0]['native_position']=[0.,0.,0.]
        with self.assertRaises(ValueError):locate(prior,views,imu,C,K)

    def test_setup_does_not_publish_a_pose_before_imu_initialization(self):
        from unittest.mock import patch
        from miniflight.replay import Calls
        from target.aigp.experiments.camera_imu import sensors
        from target.aigp.experiments.camera_imu.unknown_start import Controller
        model=dict(hover_thrust=.27,thrust_acceleration=60.,thrust_curve=[18.,65.],body_resistance_per_m=[.047]*3)
        with patch.dict(sensors.SETTINGS,position_config=model):controller=Controller(Calls(),dict(target=[5.,-1.2,-.6],yaw=-.18,delay_s=8.))
        with patch.object(controller.setup,'update',return_value=None):
            self.assertIsNone(controller.update([],[],None));self.assertIsNone(controller.update([],[],None))
        self.assertEqual(controller.gate.signal,{})

    def test_cold_view_control_uses_pixels_without_a_map_pose(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from miniflight.replay import Calls
        from miniflight.state import ImuIncrement
        from target.aigp.experiments.camera_imu import sensors,unknown_start
        model=dict(hover_thrust=.27,thrust_acceleration=60.,thrust_curve=[18.,65.],body_resistance_per_m=[.047]*3)
        with patch.dict(sensors.SETTINGS,position_config=model):gate=unknown_start.ColdGate(Calls())
        state=SimpleNamespace(time=.5,imu_increments=[ImuIncrement(.5,.01,(0.,0.,-9.81),(0.,0.,0.))])
        with patch.object(unknown_start,'vehicle',SimpleNamespace(state=state)),patch.object(gate.inertial,'acquire',return_value=False):
            gate.inertial.primary=(.5,300.);request=gate.update([],[],None);self.assertLess(request.pitch_rate,0.);self.assertGreaterEqual(request.pitch_rate,-.08)
            gate.inertial.primary=(.5,100.);request=gate.update([],[],None);self.assertGreater(request.pitch_rate,0.)
        self.assertIsNone(gate.inertial.history);self.assertIsNone(gate.inertial.rotation)

    def test_dense_detection_buffer_preserves_the_motion_baseline(self):
        from unittest.mock import patch
        from miniflight.position import PositionConfig
        from miniflight.state import ImuIncrement,Frame
        from target.aigp.experiments.camera_imu.unknown_start import ColdInertial
        from target.aigp.experiments.camera_imu import sensors
        prior,views,*_=scene();cold=ColdInertial(PositionConfig(.27,60.,thrust_curve=(18.,65.),body_resistance_per_m=(.047,)*3));cold.timing={}
        cold.update([ImuIncrement(float(t),.01,(0.,0.,-9.81),(0.,0.,0.)) for t in np.arange(0.,2.11,.01)])
        frames=[Frame(i,round(t*1e9),0.,np.zeros((360,640,3),np.uint8)) for i,t in enumerate(np.linspace(.01,2.,61))];seen=[]
        def check(prior,selected,*args):seen.extend(selected);return dict(accepted=False,reason='test')
        with patch.dict(sensors.SETTINGS,map_prior=prior),patch('target.aigp.experiments.camera_imu.unknown_start.vision.aperture_candidates',return_value=np.array(views[0]['pixels'])),patch('target.aigp.experiments.camera_imu.unknown_start.locate',side_effect=check):
            self.assertFalse(cold.acquire(frames))
        self.assertEqual(len(seen),12);self.assertGreater(seen[-1]['time']-seen[0]['time'],1.9)

    def test_fixed_map_scoring_does_not_remove_initial_position_error(self):
        from target.aigp.experiments.camera_imu.gate_filter_event_score import score
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);out=root/'scored';out.mkdir();times=[.2,.8]
            (root/'score-frame.json').write_text(json.dumps(dict(yaw_rad=0.,origin_m=[0.,0.,0.],scale=1.)))
            np.savez_compressed(root/'native-reference.npz',position_velocity=np.array([[0.,0.,0.,0.,0.,0.,0.],[1.,0.,0.,0.,0.,0.,0.]]),attitude=np.array([[0.,0.,0.,0.],[1.,0.,0.,0.]]),body_rates=np.array([[0.,0.,0.,0.],[1.,0.,0.,0.]]),motor_outputs=np.array([[0.,.27,.27,.27,.27],[1.,.27,.27,.27,.27]]))
            with gzip.open(root/'control.jsonl.gz','wt') as h:
                for t in times:h.write(json.dumps(dict(phase='ACQUIRING',imu_time=t,gate_number=1,joint_posterior=dict(known=True)))+'\n')
            with gzip.open(out/'estimates.jsonl.gz','wt') as h:
                for t in times:h.write(json.dumps(dict(time=t,position=[3.,0.,0.],velocity=[0.,0.,0.],rotation=np.eye(3).tolist(),gates={'1':dict(center=[20.,0.,0.],yaw=0.)},body_rates=[0.,0.,0.],motor_outputs=[.27]*4,position_covariance=np.eye(3).tolist(),velocity_covariance=np.eye(3).tolist(),attitude_covariance=np.eye(3).tolist()))+'\n')
            result=score(root,out);self.assertAlmostEqual(result['metrics']['full']['position']['mae'],3.)
