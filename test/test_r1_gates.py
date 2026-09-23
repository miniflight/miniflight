import ast
from contextlib import redirect_stdout
from dataclasses import replace
import io
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from target.aigp.controllers.r1_gates import Controller, GATES
from target.aigp import Race
from miniflight import Motion, Ned, PositionNed, State


class GateControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.now = 10.0
        self.enterContext(patch("target.aigp.controllers.r1_gates.time.monotonic", side_effect=lambda: self.now))
        self.enterContext(redirect_stdout(io.StringIO()))

    def state(self, position=(0, 0, 0), stamp=1):
        return State(stamp, .02, (0, 0, 0), (0, 0, 0), self.now,
                     motion=Motion(stamp, self.now, Ned(*position), Ned(0, 0, 0)))

    def update(self, index=0, position=(0, 0, 0), stamp=1, finished=-1, controller=None):
        controller = self.controller if controller is None else controller
        controller.client._race = Race(1000, 0, finished, index, 0, received_at=self.now)
        return controller.update(self.state(position, stamp))

    def test_reuses_the_existing_r1_coordinates(self):
        source = Path(__file__).resolve().parents[1] / "examples/aigp/thread_gates.py"
        tree = ast.parse(source.read_text())
        original = next(ast.literal_eval(node.value) for node in tree.body
                        if isinstance(node, ast.Assign) and node.targets[0].id == "GATES")
        self.assertEqual(GATES, original)

    def test_target_is_one_metre_beyond_gate_on_approach_line(self):
        command = self.update()
        center = GATES[0]
        target = (command.north, command.east, command.down)
        distance = math.hypot(*center)
        for value, coordinate in zip(target, center):
            self.assertAlmostEqual(value, coordinate * (1 + 1 / distance))
        self.assertAlmostEqual(math.dist(target, center), 1)

    def test_position_or_time_alone_does_not_advance_the_gate(self):
        first = self.update()
        self.assertEqual(self.update(position=GATES[0], stamp=20), first)
        beyond = (first.north, first.east, first.down)
        self.assertEqual(self.update(position=beyond, stamp=25), first)
        self.assertEqual(self.controller.gate, 0)

    def test_reported_gate_pass_advances_the_target(self):
        first = self.update()
        second = self.update(index=1, position=GATES[0], stamp=5)
        self.assertNotEqual(first, second)
        self.assertEqual(self.controller.gate, 1)
        self.assertAlmostEqual(math.dist((second.north, second.east, second.down), GATES[1]), 1)

    def test_waits_for_pose_and_race_without_commanding(self):
        for motion, race in ((None, Race(1000, 0, -1, 0, 0, self.now)),
                             (self.state().motion, None), (None, None)):
            with self.subTest(motion=motion, race=race):
                controller = Controller()
                controller.client._race = race
                missing = replace(self.state(), motion=motion)
                self.assertIsNone(controller.update(missing))
                with self.assertRaisesRegex(ValueError, "position"):
                    controller.update(replace(missing, time=11))

    def test_stale_pose_is_rejected(self):
        state = self.state()
        self.controller.client._race = Race(1000, 0, -1, 0, 0, self.now)
        self.now += 1.1
        with self.assertRaisesRegex(TimeoutError, "position"):
            self.controller.update(state)

    def test_race_packet_gap_holds_the_last_target_with_fresh_pose(self):
        first = self.update()
        self.now += 3
        position = (first.north, first.east, first.down)
        self.assertEqual(self.controller.update(self.state(position, stamp=4)), first)
        self.assertEqual(self.controller.gate, 0)

    def test_nonfinite_pose_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.update(position=(math.nan, 0, 0))

    def test_no_gate_pass_times_out(self):
        self.update()
        with self.assertRaisesRegex(TimeoutError, "gate 1"):
            self.update(stamp=47)

    def test_reset_and_wrong_course_index_are_rejected(self):
        self.update(index=2)
        with self.assertRaisesRegex(ValueError, "reset"):
            self.update(index=0)
        for index in (-1, 7, 0xffffffff):
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, "six-gate"):
                self.update(index=index, controller=Controller())

    def test_finished_race_stops(self):
        with self.assertRaises(StopIteration):
            self.update(finished=123)

    def test_last_gate_keeps_the_target_until_native_finish(self):
        target = self.update(index=len(GATES) - 1)
        self.assertEqual(self.update(index=len(GATES), stamp=100), target)
        with self.assertRaises(StopIteration):
            self.update(index=len(GATES), stamp=101, finished=123)

    def test_starting_at_a_gate_center_still_produces_a_finite_target(self):
        for index, center in enumerate(GATES):
            with self.subTest(index=index):
                command = self.update(index=index, position=center, controller=Controller())
                self.assertIsInstance(command, PositionNed)
                self.assertAlmostEqual(math.dist((command.north, command.east, command.down), center), 1)

    def test_six_gate_sequencing_in_a_point_motion_fixture(self):
        # This checks command geometry and progression, not Unreal flight dynamics.
        position, index = (0.0, 0.0, 0.0), 0
        for tick in range(2000):
            if index == len(GATES):
                with self.assertRaises(StopIteration):
                    self.update(index=index, position=position, stamp=tick * .02, finished=123)
                break
            command = self.update(index=index, position=position, stamp=tick * .02)
            target = (command.north, command.east, command.down)
            distance = math.dist(position, target)
            scale = min(1, .2 / distance) if distance else 0
            next_position = tuple(p + (t - p) * scale for p, t in zip(position, target))
            center = GATES[index]
            if position[0] >= center[0] > next_position[0]:
                fraction = (center[0] - position[0]) / (next_position[0] - position[0])
                crossing = tuple(p + (n - p) * fraction for p, n in zip(position, next_position))
                self.assertLess(math.dist(crossing, center), .01)
                index += 1
            position = next_position
        self.assertEqual(index, len(GATES))

    def test_gate_controller_has_no_transport_fields(self):
        # The baseline must remain expressible without a single MAVLink object.
        source = Path(__file__).resolve().parents[1] / "target/aigp/controllers/r1_gates.py"
        tree = ast.parse(source.read_text())
        attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertTrue(attributes.isdisjoint({"telemetry", "received_at_map", "_target", "_socket"}))


if __name__ == "__main__":
    unittest.main()
