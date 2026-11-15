import unittest

import numpy as np
from pymavlink.dialects.v20 import common

from miniflight.state import Frame
from miniflight.replay import Calls
from target.aigp.experiments.camera_imu.course import WireClient
from target.aigp.experiments.camera_imu.controller import acquisition_waypoint
from target.aigp.experiments.camera_imu import sensors


class WireBoundaryTest(unittest.TestCase):
    def test_descent_does_not_reduce_forward_velocity_target(self):
        target=sensors.velocity_target([6.,0.,10.],1.,5.,3.)
        np.testing.assert_allclose(target,[5.,0.,3.])
        np.testing.assert_allclose(sensors.velocity_target([6.,0.,-10.],1.,5.,3.),[5.,0.,-3.])
        self.assertAlmostEqual(np.linalg.norm(sensors.velocity_target([6.,0.,10.],1.,5.)),5.)

    def test_approach_distance_uses_camera_despite_inertial_position_drift(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        gate=object.__new__(sensors.GateController)
        gate.anchor=np.array([10.,0.,0.]);gate.direction=np.array([1.,0.,0.])
        gate.inertial=SimpleNamespace(position=np.array([9.,0.,0.]),velocity=np.array([2.,0.,0.]),time=1.)
        gate.last_relative_position=gate.inertial.position.copy()
        gate.last_relative_gate=np.array([5.,0.,0.]);gate.last_image_time=1.
        with patch.dict(sensors.SETTINGS,continuous_course=True):
            self.assertGreater(gate.gate_remaining(),2.5)
            gate.inertial.time=2.
            gate.inertial.position[0]+=2.
            self.assertAlmostEqual(gate.gate_remaining(),3.)
            gate.inertial.position+=1000.;gate.anchor+=1000.
            gate.last_relative_position+=1000.
            self.assertAlmostEqual(gate.gate_remaining(),3.)
            gate.inertial.time=2.5
            gate.inertial.position[0]+=1.
            self.assertLess(gate.gate_remaining(),2.5)

    def test_gate_prediction_uses_displacement_during_acceleration(self):
        from types import SimpleNamespace
        gate=object.__new__(sensors.GateController)
        gate.last_relative_gate=np.array([0.,0.,10.]);gate.last_relative_position=np.zeros(3)
        gate.inertial=SimpleNamespace(position=np.array([0.,0.,1.]),velocity=np.array([0.,0.,2.]))
        np.testing.assert_allclose(gate.gate_relative(),[0.,0.,9.])

    def test_camera_height_waypoint_preserves_the_local_frame(self):
        p=np.array([10.,-3.,-1.]);gate=np.array([40.,2.,7.])
        target=acquisition_waypoint(p,gate)
        self.assertAlmostEqual(np.linalg.norm((target-p)[:2]),6.)
        self.assertEqual(target[2],gate[2])
        shift=np.array([20.,-4.,12.])
        np.testing.assert_allclose(acquisition_waypoint(p+shift,gate+shift),target+shift)

    def test_native_navigation_packets_are_unavailable(self):
        client=WireClient(Calls())
        encoder=common.MAVLink(None,srcSystem=1,srcComponent=1)
        forbidden=[encoder.local_position_ned_encode(1000,123.,456.,789.,10.,20.,30.),
                   encoder.attitude_encode(1000,1.,2.,3.,4.,5.,6.)]
        for message in forbidden:
            self.assertIsNone(client._receive(message,1000.))
        self.assertEqual(dict(client.telemetry),{})

    def test_camera_clock_uses_only_public_sensor_arrivals(self):
        client=WireClient(Calls())
        client._begin_session()
        encoder=common.MAVLink(None,srcSystem=1,srcComponent=1)
        decoder=common.MAVLink(None)
        heartbeat=decoder.parse_char(encoder.heartbeat_encode(2,3,0,0,4).pack(encoder))
        client._receive(heartbeat,1000.)
        imu=decoder.parse_char(encoder.highres_imu_encode(2_000_000,0.,0.,-9.81,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,63).pack(encoder))
        client._receive(imu,1002.)
        frame=Frame(1,1_700_000_002_000_000_000,1002.01,np.zeros((360,640,3),np.uint8))
        aligned=client.align_frame(frame)
        self.assertAlmostEqual(aligned.time_ns*1e-9,2.01,places=5)
        self.assertEqual(aligned.id,frame.id)
        self.assertIs(aligned.bgr,frame.bgr)


if __name__=='__main__':unittest.main()
