import unittest

import numpy as np
from pymavlink.dialects.v20 import common

from miniflight.state import Frame
from miniflight.replay import Calls
from target.aigp.experiments.camera_imu.course import WireClient


class WireBoundaryTest(unittest.TestCase):
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
