from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

from miniflight import Attitude, BodyRates, Motion, Ned, State
from target.aigp.controllers import Gate
from target.aigp.controllers.r1_body_rates import Controller
from target.aigp.recording import RecordedController, recording, replay


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
        target = self.controller.target
        self.assertIsInstance(first, BodyRates)
        self.assertAlmostEqual(math.dist(target, self.gates[0].center), 1)
        moved = replace(self.state, time=20, motion=replace(self.state.motion, position=self.gates[0].center))
        self.assertIsInstance(self.controller.update(moved, 0, self.gates), BodyRates)
        self.assertEqual(self.controller.target, target)
        self.assertIsInstance(self.controller.update(moved, 1, self.gates), BodyRates)
        self.assertNotEqual(self.controller.target, target)
        self.assertEqual(self.controller.gate, 1)

    def test_missing_motion_attitude_or_geometry_returns_no_command(self):
        for state, gates in ((replace(self.state, motion=None), self.gates),
                             (replace(self.state, attitude=None), self.gates),
                             (self.state, None), (self.state, ())):
            with self.subTest(state=state, gates=gates):
                controller = Controller()
                self.assertIsNone(controller.update(state, 0, gates))
                controller.update(self.state, 0, self.gates)
                self.assertIsNone(controller.update(state, 0, gates))

    def test_last_gate_keeps_commanding_until_native_finish(self):
        self.controller.update(self.state, 1, self.gates)
        target = self.controller.target
        command = self.controller.update(self.state, 2, self.gates)
        self.assertIsInstance(command, BodyRates)
        self.assertEqual(self.controller.target, target)
        self.assertIsNone(Controller().update(self.state, 2, self.gates))

    def test_heading_is_held_from_initial_observation(self):
        self.controller.update(self.state, 0, self.gates)
        turned = replace(self.state, attitude=replace(self.state.attitude, yaw=math.pi + .1))
        command = self.controller.update(turned, 0, self.gates)
        self.assertEqual(self.controller.yaw, math.pi)
        self.assertLess(command.yaw_rate, 0)

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
