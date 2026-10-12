import unittest

from miniflight import MotorControls
from miniflight.mixer import mix
from target.aigp import MOTOR_MIX


class MixerTest(unittest.TestCase):
    def test_collective_and_independent_torque_efforts(self):
        self.assertEqual(mix(.4, (0, 0, 0), MOTOR_MIX), (.4,) * 4)
        outputs = mix(.4, (.1, -.05, .025), MOTOR_MIX)
        for actual, expected in zip(outputs, (.425, .275, .575, .325)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(sum(outputs), 1.6)

    def test_saturation_remains_visible(self):
        requested = mix(.5, (1, 0, 0), MOTOR_MIX)
        self.assertEqual(requested, (1.5, -.5, 1.5, -.5))
        with self.assertRaises(ValueError):
            MotorControls(requested)

    def test_motor_bounds_and_snapshot(self):
        values = [0, .25, .5, 1]
        command = MotorControls(values)
        values[0] = 1
        self.assertEqual(command.outputs, (0, .25, .5, 1))
        for values in ((), (-.01,), (1.01,), (float("nan"),), (float("inf"),)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                MotorControls(values)
