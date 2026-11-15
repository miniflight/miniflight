import gzip,json,tempfile,unittest
from pathlib import Path
import numpy as np
from target.aigp.experiments.camera_imu.dynamics_inputs import export
from target.aigp.experiments.camera_imu.dynamics import load,FIELDS


class PublicCalibrationTest(unittest.TestCase):
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
        from target.aigp.experiments.camera_imu.partial import relative_from_top
        width,offset=3.,1.6
        for center in [np.array([10.,2.,3.]),np.array([8.,-1.,-2.])]:
            for yaw in [-.2,.1]:
                right=np.array([-np.sin(yaw),np.cos(yaw),0.])
                points=center+np.array([-width/2*right-[0,0,offset],width/2*right-[0,0,offset]])
                camera=Rotation.from_euler('xyz',[.1,-.2,.05]).as_matrix() @ vision.CAMERA_FROM_BODY.T
                pixels=points @ camera @ vision.K.T;pixels=pixels[:,:2]/pixels[:,2,None]
                np.testing.assert_allclose(relative_from_top(pixels,camera,width,offset),center,atol=1e-10)
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
