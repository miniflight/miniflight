import json,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from miniflight.control import BodyRates
from target.aigp.experiments.camera_imu import thrust_control,sensors


class ThrustResponseTest(unittest.TestCase):
    def test_collection_finish_is_not_a_gate_crossing(self):
        from target.aigp.experiments.camera_imu import controller
        c=object.__new__(controller.GateController);c.pending=[];c.completed=[];c.phase=sensors.Phase.TRACKING
        with patch.object(sensors.GateController,'update',side_effect=StopIteration('public thrust excitation complete')):
            with self.assertRaises(StopIteration):c.update((),(),None)
        self.assertEqual(c.completed,[])

    def test_camera_response_recovers_thrust_across_different_initial_states(self):
        from target.aigp.experiments.camera_imu.thrust_response import conditional_system,fit_parameters
        t=np.linspace(-.4,1.,71);B=np.c_[np.ones(len(t)),t+.4];theta=np.array([17.,60.]);design=[];target=[]
        for i,u in enumerate([.21,.235,.26,.285,.31,.335]):
            step=.5*(np.maximum(t,0.)**2-np.maximum(t-.3,0.)**2)
            X=-np.c_[.5*.27*(t+.4)**2+(u-.27)*step,.5*.27**2*(t+.4)**2+(u*u-.27**2)*step]
            y=B @ [i*.7,i*.1-.2]+X @ theta
            a,b=conditional_system(X,y,B,t<-.025,t>=0.);design.append(a);target.append(b)
        np.testing.assert_allclose(fit_parameters(np.concatenate(design),np.concatenate(target)),theta,atol=1e-8)

    def test_native_fields_and_reused_validation_flights_are_rejected(self):
        import tempfile
        from pathlib import Path
        from target.aigp.experiments.camera_imu.thrust_response import windows,validate_provenance
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);np.savez(root/'public-inputs.npz',native_position=[1.,2.,3.])
            with self.assertRaisesRegex(ValueError,'schema'):windows(root,root/'unused.json')
        model=dict(training_inputs=dict(public_inputs_sha256='training'),baseline_sha256='prior')
        with self.assertRaisesRegex(ValueError,'separate flight'):validate_provenance(dict(public_inputs_sha256='training'),model,'prior')
        with self.assertRaisesRegex(ValueError,'Baseline changed'):validate_provenance(dict(public_inputs_sha256='heldout'),model,'changed')

    def test_pulses_require_settling_and_cancel_on_stale_vision(self):
        c=object.__new__(thrust_control.GateController)
        c.phase=sensors.Phase.TRACKING;c.hold=np.zeros(3);c.started=0.;c.pulse_at=None;c.recover_at=None
        c.steady=None;c.index=0;c.steps=thrust_control.sequence('train');c.signal={}
        c.inertial=SimpleNamespace(time=0.,position=np.zeros(3),velocity=np.zeros(3),gyro=np.zeros(3));c.measurement_time=lambda:c.inertial.time
        request=BodyRates(0.,0.,0.,.27)
        with patch.object(sensors.GateController,'drive',return_value=request):
            self.assertEqual(c.drive(None),request)
            c.inertial.time=.51;self.assertEqual(c.drive(None).thrust,.26)
            c.inertial.time=np.float64(.70);self.assertEqual(c.drive(None),request);self.assertEqual(c.index,1)
            json.dumps(c.signal)
            c.inertial.time=1.3;c.inertial.velocity[2]=.4;self.assertEqual(c.drive(None),request)
            c.inertial.velocity[:]=0.;c.inertial.time=1.4;c.drive(None)
            c.inertial.time=1.91;self.assertEqual(c.drive(None).thrust,.335)
            c.measurement_time=lambda:1.;self.assertEqual(c.drive(None),request)
            self.assertFalse(c.signal['excitation']['applied'])

    def test_validation_excites_the_same_range_with_different_edges(self):
        training=thrust_control.sequence('train');validation=thrust_control.sequence('validate')
        self.assertNotEqual(training,validation)
        self.assertEqual(sorted(set(u for u,_ in training)),sorted(set(u for u,_ in validation)))
        self.assertEqual((min(u for u,_ in training),max(u for u,_ in training)),(.21,.335))
