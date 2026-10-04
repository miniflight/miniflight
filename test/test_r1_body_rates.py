from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

from miniflight import Attitude, BodyRates, Motion, Ned, State
from target.aigp.controllers import Gate
from target.aigp.controllers.r1_body_rates import Controller
from target.aigp.controllers.r1_gates import Controller as PositionController
from target.aigp.experiments.recording import RecordedController, recording, replay


class BodyRateGateControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.gates = (Gate(0, Ned(-20, 0, -2), (1, 0, 0, 0), 2, 2),
                      Gate(1, Ned(-40, 2, -3), (1, 0, 0, 0), 2, 2))
        self.state = State(1, .02, (0, 0, 0), (0, 0, 0), 10,
                           motion=Motion(1, 10, Ned(0, 0, 0), Ned(0, 0, 0)),
                           attitude=Attitude(1, 10, 0, 0, math.pi))

    def test_only_body_rates_are_returned_while_following_native_gate_progress(self):
        first = self.controller.update(self.state, 0, self.gates)
        target = self.controller.target_position
        self.assertIsInstance(first, BodyRates)
        self.assertAlmostEqual(math.dist(target, self.gates[0].center), 1)
        moved = replace(self.state, time=20, motion=replace(self.state.motion, position=self.gates[0].center))
        self.assertIsInstance(self.controller.update(moved, 0, self.gates), BodyRates)
        self.assertEqual(self.controller.target_position, target)
        self.assertIsInstance(self.controller.update(moved, 1, self.gates), BodyRates)
        self.assertNotEqual(self.controller.target_position, target)
        self.assertEqual(self.controller.gate_index, 1)

    def test_missing_motion_attitude_or_geometry_returns_no_command(self):
        for state, gates in ((replace(self.state, motion=None), self.gates),
                             (replace(self.state, attitude=None), self.gates),
                             (self.state, None), (self.state, ())):
            with self.subTest(state=state, gates=gates):
                controller = Controller()
                self.assertIsNone(controller.update(state, 0, gates))
                controller.update(self.state, 0, self.gates)
                self.assertIsNone(controller.update(state, 0, gates))

    def test_uses_the_same_targets_as_the_position_controller_through_six_gates(self):
        position_controller = PositionController()
        gates = tuple(Gate(i, Ned(-20 * (i + 1), -i, -2 - .5 * i), (1, 0, 0, 0), 2, 2) for i in range(6))
        state = self.state
        for index in range(len(gates) + 1):
            for position in (state.motion.position, gates[min(index, len(gates) - 1)].center):
                state = replace(state, motion=replace(state.motion, position=position))
                expected = position_controller.update(state, index, gates)
                actual = self.controller.update(state, index, gates)
                self.assertIsInstance(actual, BodyRates)
                self.assertEqual(self.controller.target_position, (expected.north, expected.east, expected.down))

    def test_last_gate_keeps_commanding_until_native_finish(self):
        self.controller.update(self.state, 1, self.gates)
        target = self.controller.target_position
        command = self.controller.update(self.state, 2, self.gates)
        self.assertIsInstance(command, BodyRates)
        self.assertEqual(self.controller.target_position, target)
        self.assertIsNone(Controller().update(self.state, 2, self.gates))

    def test_heading_is_held_from_initial_observation(self):
        self.controller.update(self.state, 0, self.gates)
        turned = replace(self.state, attitude=replace(self.state.attitude, yaw=math.pi + .1))
        command = self.controller.update(turned, 0, self.gates)
        self.assertEqual(self.controller.target_yaw, math.pi)
        self.assertLess(command.yaw_rate, 0)
        self.controller.update(turned, 1, self.gates)
        self.assertEqual(self.controller.target_yaw, math.pi)

    def test_body_rate_limit_holds_during_persistent_heading_error(self):
        self.controller.update(self.state, 0, self.gates)
        turned = replace(self.state, attitude=replace(self.state.attitude, yaw=math.pi + 1))
        for tick in range(25):
            command = self.controller.update(replace(turned, time=turned.time + tick * turned.dt), 0, self.gates)
            rates = (command.roll_rate, command.pitch_rate, command.yaw_rate)
            self.assertLessEqual(math.hypot(*rates), self.controller.config.max_rate + 1e-12)

    def test_recorded_observations_and_commands_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "r1.jsonl"
            with recording(path) as record:
                controller = RecordedController(self.controller, record)
                controller.update(replace(self.state, attitude=None), 0, self.gates)
                controller.update(self.state, 0, self.gates)
                controller.update(replace(self.state, time=2), 1, self.gates)
                controller.update(replace(self.state, time=3), 2, self.gates)
            self.assertEqual(replay(Controller(), path), 4)


if __name__ == "__main__":
    unittest.main()
