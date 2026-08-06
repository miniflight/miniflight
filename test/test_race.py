"""Tests for race flight programs."""

import unittest

from miniflight.core.observation import ObservationTick
from miniflight.program import PositionNed
from miniflight.race import ThreadGatesProgram


OBSERVATION = ObservationTick(
    sequence=0,
    timestamp_ns=0,
    accel_mps2=(0.0, 0.0, 0.0),
    gyro_rad_s=(0.0, 0.0, 0.0),
    source_age_ns=0,
)


class ThreadGatesProgramTests(unittest.TestCase):
    def test_returns_the_active_gate_center(self):
        program = ThreadGatesProgram(((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)))

        self.assertEqual(program.step(OBSERVATION), PositionNed((1.0, 2.0, 3.0)))
        program.select_gate(1)
        self.assertEqual(program.step(OBSERVATION), PositionNed((4.0, 5.0, 6.0)))


if __name__ == "__main__":
    unittest.main()
