import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from miniflight.control import BodyRates
from target.aigp.experiments.camera_imu import thrust_control,sensors


class ThrustResponseTest(unittest.TestCase):
    def test_pulses_require_settling_and_cancel_on_stale_vision(self):
        c=object.__new__(thrust_control.GateController)
        c.phase=sensors.Phase.TRACKING;c.hold=np.zeros(3);c.started=0.;c.pulse_at=None;c.recover_at=None
        c.steady=None;c.index=0;c.steps=thrust_control.sequence('train');c.signal={}
        c.inertial=SimpleNamespace(time=0.,position=np.zeros(3),velocity=np.zeros(3));c.measurement_time=lambda:c.inertial.time
        request=BodyRates(0.,0.,0.,.27)
        with patch.object(sensors.GateController,'drive',return_value=request):
            self.assertEqual(c.drive(None),request)
            c.inertial.time=.51;self.assertEqual(c.drive(None).thrust,.26)
            c.inertial.time=.70;self.assertEqual(c.drive(None),request);self.assertEqual(c.index,1)
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
