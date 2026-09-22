import ast
from contextlib import redirect_stdout
import io
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from controllers.r1_gates import Controller, GATES
from miniflight import PositionNed, Race, State


class GateControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.now = 10.0
        self.enterContext(patch("controllers.r1_gates.time.monotonic", side_effect=lambda: self.now))
        self.enterContext(redirect_stdout(io.StringIO()))

    def state(self, index=0, position=(0, 0, 0), stamp=1, finished=-1):
        pose = SimpleNamespace(x=position[0], y=position[1], z=position[2])
        race = Race(1000, 0, finished, index, 0, received_at=self.now)
        return State(stamp, .02, (0, 0, 0), (0, 0, 0), None, race,
                     {"LOCAL_POSITION_NED": pose}, {"LOCAL_POSITION_NED": self.now}, ())

    def test_reuses_the_existing_r1_coordinates(self):
        source = Path(__file__).resolve().parents[1] / "examples/aigp/thread_gates.py"
        tree = ast.parse(source.read_text())
        original = next(ast.literal_eval(node.value) for node in tree.body
                        if isinstance(node, ast.Assign) and node.targets[0].id == "GATES")
        self.assertEqual(GATES, original)

    def test_target_is_one_metre_beyond_gate_on_approach_line(self):
        command = self.controller.update(self.state())
        center = GATES[0]
        target = (command.north, command.east, command.down)
        distance = math.hypot(*center)
        for value, coordinate in zip(target, center):
            self.assertAlmostEqual(value, coordinate * (1 + 1 / distance))
        self.assertAlmostEqual(math.dist(target, center), 1)

    def test_position_or_time_alone_does_not_advance_the_gate(self):
        first = self.controller.update(self.state())
        self.assertEqual(self.controller.update(self.state(position=GATES[0], stamp=20)), first)
        beyond = (first.north, first.east, first.down)
        self.assertEqual(self.controller.update(self.state(position=beyond, stamp=25)), first)
        self.assertEqual(self.controller.gate, 0)

    def test_reported_gate_pass_advances_the_target(self):
        first = self.controller.update(self.state())
        second = self.controller.update(self.state(index=1, position=GATES[0], stamp=5))
        self.assertNotEqual(first, second)
        self.assertEqual(self.controller.gate, 1)
        self.assertAlmostEqual(math.dist((second.north, second.east, second.down), GATES[1]), 1)

    def test_waits_for_pose_and_race_without_commanding(self):
        for pose, race in ((None, self.state().race), (self.state().telemetry, None), (None, None)):
            with self.subTest(pose=pose, race=race):
                controller = Controller()
                state = self.state()
                missing = State(1, .02, state.acceleration, state.gyro, None, race, pose or {}, {}, ())
                self.assertIsNone(controller.update(missing))
                late = State(11, .02, state.acceleration, state.gyro, None, race, pose or {}, {}, ())
                with self.assertRaisesRegex(ValueError, "LOCAL_POSITION_NED"):
                    controller.update(late)

    def test_stale_pose_and_race_are_rejected(self):
        state = self.state()
        self.now += 1.1
        with self.assertRaisesRegex(TimeoutError, "position"):
            self.controller.update(state)
        fresh_pose = self.state()
        self.now += 2.1
        state = State(2, .02, state.acceleration, state.gyro, None, fresh_pose.race,
                      fresh_pose.telemetry, {"LOCAL_POSITION_NED": self.now}, ())
        with self.assertRaisesRegex(TimeoutError, "race"):
            self.controller.update(state)

    def test_nonfinite_pose_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.controller.update(self.state(position=(math.nan, 0, 0)))

    def test_no_gate_pass_times_out(self):
        self.controller.update(self.state())
        with self.assertRaisesRegex(TimeoutError, "gate 1"):
            self.controller.update(self.state(stamp=47))

    def test_reset_and_wrong_course_index_are_rejected(self):
        self.controller.update(self.state(index=2))
        with self.assertRaisesRegex(ValueError, "reset"):
            self.controller.update(self.state(index=0))
        for index in (-1, 7, 0xffffffff):
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, "six-gate"):
                Controller().update(self.state(index=index))

    def test_finished_race_or_all_gates_passed_stops(self):
        for state in (self.state(finished=123), self.state(index=len(GATES))):
            with self.subTest(state=state), self.assertRaises(StopIteration):
                self.controller.update(state)

    def test_starting_at_a_gate_center_still_produces_a_finite_target(self):
        for index, center in enumerate(GATES):
            with self.subTest(index=index):
                command = Controller().update(self.state(index=index, position=center))
                self.assertIsInstance(command, PositionNed)
                self.assertAlmostEqual(math.dist((command.north, command.east, command.down), center), 1)

    def test_six_gate_sequencing_in_a_point_motion_fixture(self):
        # This checks command geometry and progression, not Unreal flight dynamics.
        position, index = (0.0, 0.0, 0.0), 0
        for tick in range(2000):
            if index == len(GATES):
                with self.assertRaises(StopIteration):
                    self.controller.update(self.state(index=index, position=position, stamp=tick * .02))
                break
            command = self.controller.update(self.state(index=index, position=position, stamp=tick * .02))
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


if __name__ == "__main__":
    unittest.main()
