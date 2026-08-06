"""Tests for the hardware-neutral flight-program boundary."""

import unittest

from miniflight.program import BodyRates, FlightProgram, PositionNed


class StubProgram(FlightProgram):
    name = "test"

    def __init__(self):
        self.started = False
        self.stopped_with = None

    def start(self):
        self.started = True

    def step(self):
        return BodyRates(9.81, 0.1, -0.2, 0.3)

    def stop(self, reason):
        self.stopped_with = reason

class FlightProgramTests(unittest.TestCase):
    def test_program_returns_command_without_universal_input(self):
        program = StubProgram()
        program.start()
        command = program.step()
        program.stop("complete")

        self.assertEqual(command, BodyRates(9.81, 0.1, -0.2, 0.3))
        self.assertTrue(program.started)
        self.assertEqual(program.stopped_with, "complete")

    def test_position_command_preserves_ned_target(self):
        command = PositionNed((1.0, -2.0, 3.0), yaw_rad=0.4)
        self.assertEqual(command.position_ned_m, (1.0, -2.0, 3.0))
        self.assertEqual(command.yaw_rad, 0.4)

    def test_body_command_rejects_invalid_collective(self):
        with self.assertRaisesRegex(ValueError, "non-negative"):
            BodyRates(-0.1, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
