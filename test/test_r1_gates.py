import ast
from dataclasses import replace
import math
from pathlib import Path
import unittest

from target.aigp.controllers import Gate
from target.aigp.controllers.r1_gates import Controller
from miniflight import Motion, Ned, PositionNed, State


class GateControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.now = 10.0
        self.gates = tuple(Gate(i, Ned(-20 * (i + 1), -i, -2 - .5 * i), (1, 0, 0, 0), 2, 2) for i in range(6))

    def state(self, position=(0, 0, 0), stamp=1):
        return State(stamp, .02, (0, 0, 0), (0, 0, 0), self.now,
                     motion=Motion(stamp, self.now, Ned(*position), Ned(0, 0, 0)))

    def update(self, index=0, position=(0, 0, 0), stamp=1, controller=None, gates=None):
        controller = self.controller if controller is None else controller
        return controller.update(self.state(position, stamp), index, self.gates if gates is None else gates)

    def test_follows_the_supplied_course(self):
        supplied = (Gate(0, Ned(7, -4, -6), (1, 0, 0, 0), 3, 3),)
        target = self.update(gates=supplied)
        self.assertAlmostEqual(math.dist((target.north, target.east, target.down), supplied[0].center), 1)
        self.assertGreater(target.north, supplied[0].center.north)
        self.assertEqual(self.update(index=1, gates=supplied), target)

    def test_target_is_one_metre_beyond_gate_on_approach_line(self):
        command = self.update()
        center = self.gates[0].center
        target = (command.north, command.east, command.down)
        distance = math.hypot(*center)
        for value, coordinate in zip(target, center):
            self.assertAlmostEqual(value, coordinate * (1 + 1 / distance))
        self.assertAlmostEqual(math.dist(target, center), 1)

    def test_position_or_time_alone_does_not_advance_the_gate(self):
        first = self.update()
        self.assertEqual(self.update(position=self.gates[0].center, stamp=20), first)
        beyond = (first.north, first.east, first.down)
        self.assertEqual(self.update(position=beyond, stamp=25), first)
        self.assertEqual(self.update(position=beyond, stamp=100), first)
        self.assertEqual(self.controller.gate, 0)

    def test_reported_gate_pass_advances_the_target(self):
        first = self.update()
        second = self.update(index=1, position=self.gates[0].center, stamp=5)
        self.assertNotEqual(first, second)
        self.assertEqual(self.controller.gate, 1)
        self.assertAlmostEqual(math.dist((second.north, second.east, second.down), self.gates[1].center), 1)

    def test_waits_for_position_without_commanding(self):
        state = replace(self.state(), motion=None)
        self.assertIsNone(self.controller.update(state, 0, self.gates))
        self.assertIsNone(self.controller.update(replace(state, time=11), 0, self.gates))

    def test_losing_position_after_start_returns_no_command(self):
        self.update()
        self.assertIsNone(self.controller.update(replace(self.state(stamp=2), motion=None), 0, self.gates))

    def test_waits_for_track_without_a_hardcoded_fallback(self):
        self.assertIsNone(self.controller.update(self.state(), 0, None))
        self.assertIsNone(self.controller.update(self.state(), 0, ()))
        self.assertIsNone(self.controller.target)
        self.assertIsInstance(self.controller.update(self.state(), 0, self.gates), PositionNed)

    def test_losing_geometry_after_start_returns_no_command(self):
        self.update()
        self.assertIsNone(self.controller.update(self.state(stamp=2), 0, None))

    def test_unchanged_gate_index_holds_the_target(self):
        first = self.update()
        self.now += 3
        position = (first.north, first.east, first.down)
        self.assertEqual(self.controller.update(self.state(position, stamp=4), 0, self.gates), first)
        self.assertEqual(self.controller.gate, 0)

    def test_nonfinite_pose_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.update(position=(math.nan, 0, 0))

    def test_last_gate_keeps_the_target_until_native_finish(self):
        target = self.update(index=len(self.gates) - 1)
        self.assertEqual(self.update(index=len(self.gates), stamp=100), target)

    def test_starting_at_a_gate_center_still_produces_a_finite_target(self):
        for index, center in enumerate(g.center for g in self.gates):
            with self.subTest(index=index):
                command = self.update(index=index, position=center, controller=Controller())
                self.assertIsInstance(command, PositionNed)
                self.assertAlmostEqual(math.dist((command.north, command.east, command.down), center), 1)

    def test_recorded_inputs_replay_without_a_connection_or_clock(self):
        samples = [
            (self.state(position, stamp=index + 1), index)
            for index, position in enumerate(((0, 0, 0), *(g.center for g in self.gates[:-1])))
        ]
        expected = [self.controller.update(state, index, self.gates) for state, index in samples]
        replay = Controller()
        for (state, index), command in zip(samples, expected):
            self.assertEqual(replay.update(state, index, self.gates), command)
        self.assertEqual(replay.gate, len(self.gates) - 1)
        self.assertEqual(self.controller.target, expected[-1])

    def test_six_gate_sequencing_in_a_point_motion_fixture(self):
        # This checks command geometry and progression, not Unreal flight dynamics.
        position, index = (0.0, 0.0, 0.0), 0
        for tick in range(2000):
            if index == len(self.gates):
                self.assertIsInstance(self.update(index=index, position=position, stamp=tick * .02), PositionNed)
                break
            command = self.update(index=index, position=position, stamp=tick * .02)
            target = (command.north, command.east, command.down)
            distance = math.dist(position, target)
            scale = min(1, .2 / distance) if distance else 0
            next_position = tuple(p + (t - p) * scale for p, t in zip(position, target))
            center = self.gates[index].center
            if position[0] >= center[0] > next_position[0]:
                fraction = (center[0] - position[0]) / (next_position[0] - position[0])
                crossing = tuple(p + (n - p) * fraction for p, n in zip(position, next_position))
                self.assertLess(math.dist(crossing, center), .01)
                index += 1
            position = next_position
        self.assertEqual(index, len(self.gates))

    def test_gate_controller_has_no_transport_fields(self):
        # The baseline must remain expressible without a single MAVLink object.
        source = Path(__file__).resolve().parents[1] / "target/aigp/controllers/r1_gates.py"
        tree = ast.parse(source.read_text())
        attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertTrue(attributes.isdisjoint({"telemetry", "received_at_map", "_target", "_socket"}))


if __name__ == "__main__":
    unittest.main()
