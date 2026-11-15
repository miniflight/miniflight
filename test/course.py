import unittest

import numpy as np
from pymavlink.dialects.v20 import common

from miniflight.state import Frame
from miniflight.replay import Calls
from target.aigp.experiments.camera_imu.course import WireClient, alignment_step
from target.aigp.experiments.camera_imu.controller import acquisition_waypoint
from target.aigp.experiments.camera_imu import sensors
from target.aigp.experiments.camera_imu.motion import GateMotion
from target.aigp.experiments.camera_imu import vision
from target.aigp.experiments.camera_imu.score import scored_gate_passes
import cv2


class WireBoundaryTest(unittest.TestCase):
    def test_finish_freeze_does_not_hide_an_earlier_missed_gate(self):
        gates=[dict(native_pass=True,crossing_imu_time_s=float(i)) for i in range(6)]
        gates[-1]['crossing_imu_time_s']=None
        self.assertFalse(scored_gate_passes(gates))
        self.assertTrue(scored_gate_passes(gates,completed_finish=True))
        gates[1]['crossing_imu_time_s']=None
        self.assertFalse(scored_gate_passes(gates,completed_finish=True))

    def test_partial_bearing_overrides_an_incorrect_height_prediction(self):
        self.assertGreater(alignment_step(-5.,1.,350.,1.),0.)
        self.assertLess(alignment_step(5.,1.,100.,-1.),0.)
        self.assertEqual(alignment_step(5.,.1,100.,-1.),5.)

    def test_real_gate_measurement_can_repair_a_drifted_imu_position(self):
        from unittest.mock import patch
        from common.math import Quaternion
        image=np.zeros((360,640,3),np.uint8)
        cv2.rectangle(image,(200,130),(360,290),(0,0,220),-1)
        cv2.rectangle(image,(220,150),(340,270),(0,0,0),-1)
        corners=np.array([[219.5,149.5],[340.5,149.5],[340.5,270.5],[219.5,270.5]])
        q=vision.CAMERA_FROM_BODY.T
        with patch.dict(sensors.SETTINGS,continuous_course=True,camera_correction=True,camera_position_min_side_px=20.):
            controller=sensors.GateController()
            controller.locked=0;controller.count=3;controller.anchor=np.array([20.,0.,0.])
            controller.direction=np.array([1.,0.,0.]);controller.camera_fusion_started=True
            controller.previous=(np.zeros(3),q,corners,vision.best_pose(corners))
            inertial=controller.inertial;inertial.rotation=Quaternion(1.,0.,0.,0.)
            inertial.time=1.;inertial.times=[.9,1.]
            inertial.positions=[np.zeros(3),np.zeros(3)];inertial.velocities=[np.zeros(3),np.zeros(3)]
            inertial.rotations=[q.copy(),q.copy()]
            controller.observe(Frame(2,1_000_000_000,1001.,image),1.,np.zeros(3),q)
            self.assertEqual(controller.last_image_id,2)
            self.assertGreater(inertial.position[0],3.)

    def test_blue_guidance_edge_cannot_replace_the_red_aperture(self):
        image=np.zeros((360,640,3),np.uint8)
        cv2.rectangle(image,(200,130),(360,290),(0,0,220),-1)
        cv2.rectangle(image,(220,150),(340,270),(0,0,0),-1)
        cv2.line(image,(222,250),(338,250),(255,255,0),12)
        predicted=np.array([[220.,150.],[340.,150.],[340.,270.],[220.,270.]])+[0.,-8.]
        corners=vision.refine(image,predicted,minimum_supported=4,red_edges=True)
        self.assertIsNotNone(corners)
        np.testing.assert_allclose(corners.mean(axis=0),[280.,210.],atol=.6)
        np.testing.assert_allclose(corners[2:,1],270.5,atol=.6)

    def test_bearing_motion_rejects_correlated_range_jitter(self):
        observer=GateMotion(1.)
        for t in np.linspace(0.,.5,31):
            position=np.array([-40.+5*t,-2.+.3*t,-10.+2*t])
            raw=observer.observe(float(t),position*(1.08 if t>=.25 else 1.))
        angular=observer.bearing_motion([1.,0.,0.])
        measured=np.array(angular['bearing'])*5.-angular['range_m']*np.array(angular['bearing_rate'])
        np.testing.assert_allclose(measured,[.3,2.],atol=.1)
        self.assertGreater(abs(raw['velocity'][2]-2.),.5)

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
