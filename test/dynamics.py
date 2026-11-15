import gzip,json,tempfile,unittest
from pathlib import Path
import numpy as np
from target.aigp.experiments.camera_imu.dynamics_inputs import export
from target.aigp.experiments.camera_imu.dynamics import load,FIELDS


class PublicCalibrationTest(unittest.TestCase):
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
