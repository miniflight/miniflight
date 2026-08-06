"""Tests for direct VQ1 MAVLink submission."""

import unittest

from miniflight.program import (
    AccelerationNed,
    Attitude,
    BodyRates,
    MotorThrusts,
    PositionNed,
    VelocityNed,
)
from target.vq1 import Vq1Link


class FakeMavlink:
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1 << 0
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 1 << 1
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 1 << 2
    POSITION_TARGET_TYPEMASK_VX_IGNORE = 1 << 3
    POSITION_TARGET_TYPEMASK_VY_IGNORE = 1 << 4
    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 1 << 5
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 1 << 6
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 1 << 7
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 1 << 8
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1 << 10
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 1 << 11
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE = 1 << 7
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE = 1 << 0
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 1 << 1
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE = 1 << 2
    MAV_FRAME_LOCAL_NED = 1


class Recorder:
    def __init__(self):
        self.position = None
        self.attitude = None

    def set_position_target_local_ned_send(self, *values):
        self.position = values

    def set_attitude_target_send(self, *values):
        self.attitude = values


class FakeLink:
    def __init__(self):
        self.mav = Recorder()
        self.target_system = 7
        self.target_component = 8


class Vq1LinkTests(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.link_transport = Vq1Link(
            self.link,
            boot_ms=1000,
            mavlink=FakeMavlink,
            clock_ms=lambda: 1250,
        )

    def test_position_ned_passes_through_without_lowering(self):
        self.link_transport.send(PositionNed((1.0, -2.0, 3.0)))

        values = self.link.mav.position
        self.assertEqual(values[0:4], (250, 7, 8, FakeMavlink.MAV_FRAME_LOCAL_NED))
        self.assertEqual(values[5:8], (1.0, -2.0, 3.0))

    def test_body_rates_pass_through_with_target_collective_conversion(self):
        self.link_transport.send(BodyRates(9.80665, 0.1, -0.2, 0.3))

        values = self.link.mav.attitude
        self.assertEqual(values[5:8], (0.1, -0.2, 0.3))
        self.assertAlmostEqual(values[8], 0.264)

    def test_velocity_ned_passes_through_without_lowering(self):
        self.link_transport.send(VelocityNed((4.0, 5.0, -6.0)))

        self.assertEqual(self.link.mav.position[8:11], (4.0, 5.0, -6.0))

    def test_acceleration_ned_passes_through_without_lowering(self):
        self.link_transport.send(AccelerationNed((0.1, -0.2, 0.3)))

        self.assertEqual(self.link.mav.position[11:14], (0.1, -0.2, 0.3))

    def test_attitude_passes_through_without_lowering(self):
        self.link_transport.send(Attitude((1.0, 0.0, 0.0, 0.0), 9.80665))

        self.assertEqual(self.link.mav.attitude[4], [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(self.link.mav.attitude[8], 0.264)

    def test_unsupported_command_is_rejected(self):
        with self.assertRaisesRegex(TypeError, "VQ1 does not accept MotorThrusts"):
            self.link_transport.send(MotorThrusts((0.1, 0.1, 0.1, 0.1)))

if __name__ == "__main__":
    unittest.main()
