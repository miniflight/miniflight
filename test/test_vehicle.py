from dataclasses import FrozenInstanceError, replace
import math
import unittest
from unittest.mock import Mock

from miniflight import BodyRates, Motion, Ned, PositionNed, State, Vehicle, VelocityNed
from target import Target


class VehicleTest(unittest.TestCase):
    def setUp(self):
        self.target = Mock(spec=Target)
        self.target.commands = frozenset((BodyRates, PositionNed, VelocityNed))
        self.vehicle = Vehicle(self.target)
        self.state = State(2, .02, (1, 2, 3), (.1, .2, .3), 10,
                           motion=Motion(1.9, 9.9, Ned(1, 2, -3), Ned(4, 5, -6)))
        self.target.read.return_value = self.state

    def test_construction_and_connect_do_not_read_or_arm(self):
        self.assertIsNone(self.vehicle.state)
        self.assertIsNone(self.vehicle.position)
        self.assertIsNone(self.vehicle.velocity)
        self.target.assert_not_called()
        self.vehicle.connect()
        self.target.connect.assert_called_once()
        self.target.read.assert_not_called()
        self.target.arm.assert_not_called()
        self.target.send.assert_not_called()

    def test_read_publishes_one_snapshot_and_properties_never_read(self):
        self.assertIs(self.vehicle.read(timeout=.2), self.state)
        self.target.read.assert_called_once_with(timeout=.2)
        self.assertIs(self.vehicle.state, self.state)
        self.assertEqual(self.vehicle.position, (1, 2, -3))
        self.assertEqual(self.vehicle.velocity, (4, 5, -6))
        self.target.read.assert_called_once()

    def test_missing_observations_remain_missing(self):
        self.target.read.return_value = replace(self.state, motion=None)
        state = self.vehicle.read()
        self.assertIsNone(self.vehicle.position)
        self.assertIsNone(self.vehicle.velocity)
        self.assertIsNone(state.frame)
        self.assertIsNone(state.attitude)
        self.assertIsNone(state.motors)
        self.assertFalse(hasattr(state, "race"))
        self.assertFalse(hasattr(state, "telemetry"))

    def test_samples_are_immutable_and_keep_independent_timestamps(self):
        first = self.vehicle.read()
        self.target.read.return_value = replace(first, time=2.02, received_at=10.02)
        second = self.vehicle.read()
        self.assertEqual(first.time, 2)
        self.assertEqual(second.motion.received_at, 9.9)
        with self.assertRaises(FrozenInstanceError):
            first.dt = 1
        with self.assertRaises(FrozenInstanceError):
            first.motion.position = Ned(0, 0, 0)

    def test_timeout_does_not_refresh_the_last_sample(self):
        self.vehicle.read()
        self.target.read.side_effect = TimeoutError("IMU")
        with self.assertRaises(TimeoutError):
            self.vehicle.read()
        self.assertIs(self.vehicle.state, self.state)
        self.assertEqual(self.vehicle.state.received_at, 10)

    def test_raw_transport_objects_cannot_become_vehicle_state(self):
        self.target.read.return_value = {"HIGHRES_IMU": object()}
        with self.assertRaisesRegex(TypeError, "return State"):
            self.vehicle.read()
        self.assertIsNone(self.vehicle.state)

    def test_commands_are_passed_to_the_target_without_hidden_loops(self):
        for command in (BodyRates(.1, -.2, .3, .4), PositionNed(1, 2, -3), VelocityNed(4, 5, -6)):
            self.vehicle.send(command)
            self.assertIs(self.target.send.call_args.args[0], command)
        self.assertEqual(self.target.send.call_count, 3)
        self.target.read.assert_not_called()
        self.target.arm.assert_not_called()

    def test_convenience_methods_use_the_same_command_values(self):
        self.vehicle.body_rates(.1, -.2, .3, .4)
        self.vehicle.position_ned(1, 2, -3)
        self.vehicle.velocity_ned(4, 5, -6)
        self.assertEqual([call.args[0] for call in self.target.send.call_args_list],
                         [BodyRates(.1, -.2, .3, .4), PositionNed(1, 2, -3), VelocityNed(4, 5, -6)])

    def test_unsupported_planes_fail_without_emulation_or_io(self):
        self.target.commands = frozenset((BodyRates,))
        self.assertEqual(self.vehicle.commands, frozenset((BodyRates,)))
        for command in (PositionNed(0, 0, 0), VelocityNed(0, 0, 0)):
            with self.subTest(command=command), self.assertRaises(NotImplementedError):
                self.vehicle.send(command)
        self.target.send.assert_not_called()
        self.target.arm.assert_not_called()
        self.target.read.assert_not_called()

    def test_invalid_commands_are_rejected_before_io(self):
        for values in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf)):
            for method in (self.vehicle.position_ned, self.vehicle.velocity_ned):
                with self.subTest(method=method, values=values), self.assertRaises(ValueError):
                    method(*values)
        for command in ((0, 0, 0), object(), None):
            with self.subTest(command=command), self.assertRaises(TypeError):
                self.vehicle.send(command)
        self.target.send.assert_not_called()

    def test_disconnect_clears_state_without_implicitly_arming_or_disarming(self):
        self.vehicle.read()
        self.vehicle.disconnect()
        self.target.disconnect.assert_called_once()
        self.target.arm.assert_not_called()
        self.target.disarm.assert_not_called()
        self.assertIsNone(self.vehicle.state)

    def test_arm_and_disarm_remain_explicit(self):
        self.vehicle.arm()
        self.vehicle.disarm()
        self.target.arm.assert_called_once()
        self.target.disarm.assert_called_once()


if __name__ == "__main__":
    unittest.main()
