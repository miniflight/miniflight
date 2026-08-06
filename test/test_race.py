"""Tests for race flight programs."""

import unittest

from miniflight.program import PositionNed
from miniflight.race import ThreadGatesProgram


class ThreadGatesProgramTests(unittest.TestCase):
    def test_returns_the_active_gate_center(self):
        program = ThreadGatesProgram(((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)))

        self.assertEqual(program.step(), PositionNed((1.0, 2.0, 3.0)))
        program.select_gate(1)
        self.assertEqual(program.step(), PositionNed((4.0, 5.0, 6.0)))


if __name__ == "__main__":
    unittest.main()
