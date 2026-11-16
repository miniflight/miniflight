import gzip,json,tempfile,unittest
from pathlib import Path
import numpy as np
from target.aigp.experiments.camera_imu.dynamics_inputs import export
from target.aigp.experiments.camera_imu.dynamics import load,FIELDS


class PublicCalibrationTest(unittest.TestCase):
    def test_replay_uses_only_its_recorded_configuration_and_restores_the_caller(self):
        from unittest.mock import patch
        from target.aigp.experiments.camera_imu import runner,sensors
        metadata=dict(process='public_camera_imu_course',revision='test',settings=dict(continuous_course=True))
        def inspect(*args,**kwargs):
            self.assertFalse(sensors.SETTINGS.get('upright_attitude',False));self.assertFalse(sensors.SETTINGS.get('image_time_uncertainty',False))
            self.assertNotIn('unrecorded_option',sensors.SETTINGS)
            raise RuntimeError('configuration checked')
        with tempfile.TemporaryDirectory() as directory,patch.dict(sensors.SETTINGS,upright_attitude=True,image_time_uncertainty=True,unrecorded_option=7):
            before=dict(sensors.SETTINGS);path=Path(directory)/'flight.jsonl';path.write_text(json.dumps(dict(event='metadata',metadata=metadata))+'\n')
            with patch.object(runner,'execute',side_effect=inspect),self.assertRaisesRegex(RuntimeError,'configuration checked'):runner.replay(path)
            self.assertEqual(sensors.SETTINGS,before)

    def test_public_course_rejects_native_fitted_default_before_launch(self):
        from unittest.mock import patch
        from target.aigp.experiments.camera_imu import runner,sensors
        with patch.dict(sensors.SETTINGS,position_config=None),patch.object(runner.native,'launch',side_effect=AssertionError('launched')):
            with self.assertRaisesRegex(ValueError,'publicly identified'):
                runner.public_course(Path('/unused'),-.5)
            with self.assertRaisesRegex(ValueError,'publicly identified'):
                sensors.GateController()

    def test_archived_process_replay_names_its_recorded_revision(self):
        from unittest.mock import patch
        from target.aigp.experiments.camera_imu import runner,sensors
        settings=dict(sensors.SETTINGS)
        for process,archived in [('camera_imu',{}),('public_camera_imu_course',dict(continuous_course=True,route_prior={'points':{}}))]:
            with tempfile.TemporaryDirectory() as directory,patch.object(runner,'execute',side_effect=AssertionError('controller executed')):
                path=Path(directory)/'flight.jsonl'
                path.write_text(json.dumps(dict(metadata=dict(process=process,revision='old-source-revision',settings=archived)))+'\n')
                with self.assertRaisesRegex(ValueError,'recorded revision old-source-revision'):
                    runner.replay(path)
        self.assertEqual(sensors.SETTINGS,settings)

    def test_client_delay_uses_receipt_and_send_return_without_exposure_claim(self):
        from target.aigp.experiments.camera_imu.pipeline_latency import measure
        data=dict(motor_sent_host_time=np.array([9.,10.05,10.15,11.]),imu_received_at=np.array([10.04,10.14]),
            frame_id=np.array([1,2]),frame_received_at=np.array([10.,10.1]))
        # Two constraints from one frame count once, after the last constraint is ready.
        observations=[dict(image_id=1,host_time=10.01),dict(image_id=1,host_time=10.03),
            dict(image_id=2,host_time=10.12),dict(image_id=3,host_time=10.13)]
        report,rows=measure(data,observations,10.,10.2)
        self.assertEqual(len(rows),2);self.assertEqual(report['missing_frame_receipts'],1)
        self.assertAlmostEqual(report['receipt_to_send_return_ms']['median'],50.)
        self.assertAlmostEqual(report['receipt_to_candidate_ms']['median'],25.)
        self.assertAlmostEqual(report['latest_imu_receipt_to_send_return_ms']['median'],10.)
        self.assertIsNone(report['exposure_to_actuation_ms'])
        unarmed,_=measure(data,observations)
        self.assertEqual(unarmed['candidate_images'],0)
        self.assertEqual(unarmed['latest_imu_receipt_to_send_return_ms']['count'],0)

    def test_pipeline_reports_wall_cpu_and_host_time_separately(self):
        from target.aigp.experiments.camera_imu.pipeline import analyze
        rows=[dict(event='call',name='arm',host_time=1000.,arguments=dict(armed=True)),
              dict(event='call',name='send',host_time=1000.5,arguments=dict(kind='MotorControls',outputs=[.25]*4)),
              dict(event='profile',scope='controller',host_time=1001.,stages=dict(controller_total=dict(wall_ns=3000000,thread_cpu_ns=2000000,calls=1)))]
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'flight.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'score.json').write_text(json.dumps(dict(passed_without_reset=False)))
            (root/'protocol.json').write_text(json.dumps(dict(revision='test',settings={})))
            report=analyze(root);stage=report['timing']['inclusive_stages']['controller_total']
            self.assertEqual(stage['wall_ms']['median'],3.)
            self.assertEqual(stage['thread_cpu_ms']['median'],2.)
            self.assertIsNone(report['input_contract']['true_camera_exposure_time_ns'])
            self.assertIsNone(report['control']['host_command_interval_ms']['median'])
            self.assertTrue((root/'public-inputs.npz').exists())

    def test_pipeline_input_export_drops_native_and_fused_fields(self):
        from target.aigp.experiments.camera_imu.pipeline import collect
        rows=[dict(event='public_imu',samples=[dict(time=1.,received_at=1001.,acceleration=[0.,0.,-9.81],gyro=[.1,.2,.3],native_position=[99]*3)]),
              dict(event='public_camera',image_id=3,header_time_ns=1000000000000,estimated_time_ns=1000000000,received_at=1001.,shape=[360,640,3]),
              dict(event='call',name='send',host_time=1001.,arguments=dict(kind='MotorControls',outputs=[.25]*4)),
              dict(event='native_pose',position=[99]*3,velocity=[99]*3),
              dict(event='update',signals=dict(position=[99]*3,velocity=[99]*3))]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'flight.jsonl';path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            data,_=collect(path)
            self.assertFalse(any('position' in name or 'velocity' in name for name in data))
            np.testing.assert_array_equal(data['gyro'],[[.1,.2,.3]])
            np.testing.assert_array_equal(data['motor_outputs'],[[.25]*4])

    def test_pixel_identity_precedes_depth_for_full_and_partial_views(self):
        import cv2
        from target.aigp.experiments.camera_imu.partial import same_red_component
        image=np.zeros((360,640,3),np.uint8)
        cv2.rectangle(image,(100,120),(300,359),(0,0,220),-1)
        cv2.rectangle(image,(125,145),(275,335),(0,0,0),-1)
        aperture=np.array([[125.,145.],[275.,145.],[275.,335.],[125.,335.]])
        top=np.array([[100.,120.],[300.,120.]])
        self.assertTrue(same_red_component(image,aperture,top))
        cv2.rectangle(image,(170,195),(230,275),(0,0,220),-1)
        cv2.rectangle(image,(178,203),(222,267),(0,0,0),-1)
        farther=np.array([[178.,203.],[222.,203.],[222.,267.],[178.,267.]])
        self.assertFalse(same_red_component(image,farther,top))

    def test_nearer_partial_target_needs_distinct_images_and_rejects_farther_aperture(self):
        from unittest.mock import patch
        from common.math import Quaternion
        from miniflight.state import Frame
        from target.aigp.experiments.camera_imu import sensors,controller,vision
        camera=vision.CAMERA_FROM_BODY.T;near=np.array([15.,0.,3.]);far=np.array([40.,0.,2.])
        top_world=near+np.array([[0.,-1.5,-1.6],[0.,1.5,-1.6]])
        projected=top_world @ camera @ vision.K.T;top=projected[:,:2]/projected[:,2,None]
        far_opening=far+.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])
        far_xyz=far_opening @ camera;pixels=far_xyz @ vision.K.T;far_corners=pixels[:,:2]/pixels[:,2,None]
        far_pose=(.1,far_xyz,far @ camera)
        with patch.dict(sensors.SETTINGS,position_config=dict(hover_thrust=.27,thrust_acceleration=60.),partial_gate=True,continuous_course=True):
            c=controller.GateController();c.phase=sensors.Phase.TRACKING;c.gate_number=4;c.completed=[{}, {}, {}]
            c.inertial.rotation=Quaternion();c.inertial.time=1.2;c.inertial.times=[.5,1.,1.1,1.2]
            c.inertial.positions=[np.zeros(3) for _ in c.inertial.times];c.inertial.velocities=[np.zeros(3) for _ in c.inertial.times]
            c.inertial.rotations=[camera.copy() for _ in c.inertial.times]
            c.anchor=far.copy();c.direction=np.array([1.,0.,0.]);c.locked=c.track=0;c.count=3
            c.last_relative_gate=far.copy();c.last_relative_position=np.zeros(3)
            c.last_image_id=10;c.last_image_time=.5;c.full_image_time=.5;c.last_corners=far_corners.copy()
            c.previous=(np.zeros(3),camera.copy(),far_corners.copy(),far_pose);c.used_frames=[8,9,10]
            c.camera_observation=dict(capture_time=.5);c.centers=[far.copy() for _ in range(3)]
            c.rim_template.frozen=dict(width_m=3.,center_to_top_m=1.6,samples=10,fractional_spread=0.)
            image=np.zeros((360,640,3),np.uint8)
            with patch.object(sensors,'outer_top',return_value=(top,.1)):
                frame=Frame(21,1_000_000_000,1001.,image)
                for _ in range(3):c.observe_partial(frame,1.,camera,np.zeros(3),far)
                np.testing.assert_allclose(c.anchor,far)
                for image_id,stamp in [(22,1.1),(23,1.2)]:
                    c.observe_partial(Frame(image_id,round(stamp*1e9),1000+stamp,image),stamp,camera,np.zeros(3),far)
            np.testing.assert_allclose(c.anchor,near,atol=1e-9);np.testing.assert_allclose(c.inertial.position,0.,atol=1e-9)
            self.assertIsNone(c.last_image_id);self.assertIsNone(c.full_image_time);self.assertEqual(len(c.completed),3)
            self.assertEqual(c.partial_retargets,1);np.testing.assert_allclose(c.lookahead['center'],far)
            c.inertial.time=1.3;c.inertial.times.append(1.3);c.inertial.positions.append(np.zeros(3));c.inertial.velocities.append(np.zeros(3));c.inertial.rotations.append(camera.copy())
            with patch.object(sensors,'outer_top',return_value=None),patch.object(vision,'refine',return_value=None),patch.object(vision,'nearest_aperture',return_value=far_corners),patch.object(vision,'best_pose',return_value=far_pose):
                c.observe(Frame(24,1_300_000_000,1001.3,image),1.3,np.zeros(3),camera)
            np.testing.assert_allclose(c.anchor,near,atol=1e-9);self.assertIsNone(c.last_image_id)

    def test_partial_measurement_cannot_keep_a_different_gate_fresh(self):
        from target.aigp.experiments.camera_imu import sensors
        c=object.__new__(sensors.GateController);c.gate_number=3;c.last_image_time=1.
        c.camera_observation=dict(capture_time=1.)
        c.partial_observation=dict(gate_number=3,capture_time=2.)
        self.assertEqual(c.measurement_time(),2.)
        c.gate_number=4
        self.assertEqual(c.measurement_time(),1.)

    def test_visual_startup_waits_for_imu_before_processing_a_camera_frame(self):
        from unittest.mock import patch,Mock
        from types import SimpleNamespace
        from common.math import Quaternion
        from miniflight.state import Frame
        from target.aigp.experiments.camera_imu import sensors,vision
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=1.)
        with patch.dict(sensors.SETTINGS,position_config=dict(hover_thrust=.27,thrust_acceleration=60.),visual_initialization=True),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            c=sensors.GateController();c.phase=sensors.Phase.SHIFTING;c.setup_started=1.
            c.inertial.rotation=Quaternion();c.inertial.time=1.;c.observe=Mock()
            c.inertial.camera_pose=lambda t:None if t>c.inertial.time else (np.zeros(3),vision.CAMERA_FROM_BODY.T)
            frame=Frame(1,1_100_000_000,1001.1,np.zeros((360,640,3),np.uint8))
            self.assertIsNone(c.update((),(frame,),None));c.observe.assert_not_called()
            state.time=c.inertial.time=1.2
            self.assertIsNone(c.update((),(),None));c.observe.assert_called_once()
            self.assertEqual(c.pending,[])

    def test_clipped_frame_retains_two_measured_top_corners(self):
        import cv2
        from target.aigp.experiments.camera_imu.partial import outer_top
        image=cv2.imread(str(Path(__file__).parent/'fixtures/clipped_gate.jpg'))
        top,residual=outer_top(image)
        np.testing.assert_allclose(top,[[197.,319.],[354.,319.]],atol=1.)
        self.assertLess(residual,.8)
        self.assertIsNone(outer_top(np.zeros_like(image)))

    def test_rim_dimensions_come_from_a_complete_pixel_view(self):
        from scipy.spatial.transform import Rotation
        from target.aigp.experiments.camera_imu import vision
        from target.aigp.experiments.camera_imu.partial import measure_rim
        camera=Rotation.from_euler('xyz',[.1,-.1,.05]).as_matrix() @ vision.CAMERA_FROM_BODY.T
        opening=np.array([[10.,-.75,-.75],[10.,.75,-.75],[10.,.75,.75],[10.,-.75,.75]])
        outer=np.array([[10.,-1.5,-1.6],[10.,1.5,-1.6]])
        def project(points):
            pixels=points @ camera @ vision.K.T
            return pixels[:,:2]/pixels[:,2,None]
        pose=vision.best_pose(project(opening))
        np.testing.assert_allclose(measure_rim(project(outer),pose),[3.,1.6],atol=1e-9)

    def test_two_top_corners_recover_metric_position_with_gravity(self):
        from scipy.spatial.transform import Rotation
        from target.aigp.experiments.camera_imu import vision
        from target.aigp.experiments.camera_imu.partial import relative_from_top,predicted_aperture
        width,offset=3.,1.6
        for center in [np.array([10.,2.,3.]),np.array([8.,-1.,-2.])]:
            for yaw in [-.2,.1]:
                right=np.array([-np.sin(yaw),np.cos(yaw),0.])
                points=center+np.array([-width/2*right-[0,0,offset],width/2*right-[0,0,offset]])
                camera=Rotation.from_euler('xyz',[.1,-.2,.05]).as_matrix() @ vision.CAMERA_FROM_BODY.T
                pixels=points @ camera @ vision.K.T;pixels=pixels[:,:2]/pixels[:,2,None]
                np.testing.assert_allclose(relative_from_top(pixels,camera,width,offset),center,atol=1e-10)
                predicted,pose=predicted_aperture(pixels,camera,width,offset)
                opening=center+.75*np.array([-right-[0,0,1],right-[0,0,1],right+[0,0,1],-right+[0,0,1]])
                reference=opening @ camera @ vision.K.T
                np.testing.assert_allclose(predicted,reference[:,:2]/reference[:,2,None],atol=1e-9)
                self.assertIsNone(pose[0])
        points=np.array([[8.,-1.5,0.],[8.,1.5,0.]])
        pixels=points @ vision.CAMERA_FROM_BODY.T @ vision.K.T;pixels=pixels[:,:2]/pixels[:,2,None]
        self.assertIsNone(relative_from_top(pixels,vision.CAMERA_FROM_BODY.T,width,offset))

    def test_visual_setup_produces_no_command_without_a_gate(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from common.math import Quaternion
        from target.aigp.experiments.camera_imu import sensors
        config=dict(hover_thrust=.27,thrust_acceleration=60.)
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=1.)
        with patch.dict(sensors.SETTINGS,position_config=config,visual_initialization=True),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            controller=sensors.GateController();controller.phase=sensors.Phase.SHIFTING
            controller.inertial.rotation=Quaternion();controller.inertial.time=1.;controller.setup_started=1.
            self.assertIsNone(controller.update((),(),None))
            self.assertIsNone(controller.locked)

    def test_visual_setup_origin_change_preserves_the_observed_gate(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from common.math import Quaternion
        from target.aigp.experiments.camera_imu import sensors,vision
        state=SimpleNamespace(motion=None,attitude=None,imu_increments=(),time=1.)
        with patch.dict(sensors.SETTINGS,position_config=dict(hover_thrust=.27,thrust_acceleration=60.),visual_initialization=True),patch.object(sensors,'vehicle',SimpleNamespace(state=state)):
            c=sensors.GateController();c.phase=sensors.Phase.SHIFTING;c.locked=1;c.setup_started=0.;c.setup_steady=.5
            c.inertial.rotation=Quaternion();c.inertial.time=1.;c.inertial.position=np.array([.2,-.5,-1.])
            c.anchor=np.array([20.,1.,2.]);c.direction=np.array([1.,0.,0.]);c.last_image_time=1.;c.last_image_id=1
            c.last_corners=np.array([[200.,150.],[280.,150.],[280.,230.],[200.,230.]])
            relative=c.anchor-c.inertial.position;c.last_relative_gate=relative.copy();c.last_relative_position=c.inertial.position.copy()
            c.previous=(c.inertial.position.copy(),vision.CAMERA_FROM_BODY.T,np.zeros((4,2)),None)
            c.last_gate_corners=np.zeros((4,3));c.inertial.rotations=[vision.CAMERA_FROM_BODY.T.copy()]
            with patch.object(c,'drive',return_value=None):c.update((),(),None)
            np.testing.assert_allclose(c.anchor,relative);np.testing.assert_allclose(c.gate_relative(),relative)
            self.assertEqual(c.phase,sensors.Phase.TRACKING)
            self.assertEqual(c.inertial.times,[1.])

    def test_supplied_model_never_reads_the_native_calibration_fixture(self):
        from unittest.mock import patch
        from target.aigp.experiments.camera_imu import sensors
        config=dict(hover_thrust=.27,thrust_acceleration=60.,thrust_curve=[10.,100.],body_resistance_per_m=[.05,.05,.1])
        with patch.dict(sensors.SETTINGS,position_config=config),patch.object(Path,'read_text',side_effect=AssertionError('native fixture read')):
            controller=sensors.GateController()
        self.assertEqual(controller.config.thrust_curve,[10.,100.])
        self.assertEqual(controller.config.body_resistance_per_m,[.05,.05,.1])

    def test_fused_or_native_motion_cannot_change_the_exported_inputs(self):
        rows=[dict(imu_time=1.,camera_orientation=np.eye(3).tolist(),motor_commands=[.25]*4,gate_number=1,
            camera_observation=dict(image_id=7,capture_time=1.,corners=[[0,0],[1,0],[1,1],[0,1]]),
            position=[999,999,999],velocity=[999]*3,native_position=[999]*3)]
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'control.gz'
            def write():
                with gzip.open(source,'wt') as stream:stream.write(json.dumps(rows[0])+'\n')
            write();export(source,root/'first.npz');first=load(root/'first.npz')
            rows[0].update(position=[-999]*3,velocity=[-999]*3,native_position=[-999]*3)
            write();export(source,root/'second.npz');second=load(root/'second.npz')
            for field in FIELDS:np.testing.assert_array_equal(first[field],second[field])

    def test_fitter_rejects_added_reference_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'bad.npz';data={k:np.zeros(1) for k in FIELDS};data['native_velocity']=np.zeros(3)
            np.savez(path,**data)
            with self.assertRaisesRegex(ValueError,'public-data schema'):load(path)
