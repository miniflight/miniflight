from dataclasses import replace
import unittest
from unittest.mock import patch

from examples.aigp.velocity_ned import Controller
from miniflight import State, VelocityNed
from target.aigp.simulator import SimulatorClient


class VelocityExampleTest(unittest.TestCase):
    def test_commands_use_simulator_time_and_stop_after_three_seconds(self):
        controller = Controller()
        state = State(10, .02, (0, 0, 0), (0, 0, 0), 100)
        self.assertEqual(controller.update(state, 0, None), VelocityNed(-1, 0, 0))
        self.assertEqual(controller.update(replace(state, time=11.9), 0, None), VelocityNed(-1, 0, 0))
        self.assertEqual(controller.update(replace(state, time=12), 0, None), VelocityNed(0, 0, 0))
        with self.assertRaises(StopIteration):
            controller.update(replace(state, time=13), 0, None)

    def test_construction_and_update_do_not_create_a_connection(self):
        with patch.object(SimulatorClient, "__init__", side_effect=AssertionError("unexpected I/O")):
            controller = Controller()
            controller.update(State(0, 0, (0, 0, 0), (0, 0, 0), 0), 0, None)
