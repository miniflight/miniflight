from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

from miniflight import BodyRates
from miniflight.position import PositionConfig, position_control


class PositionControlTest(unittest.TestCase):
    def setUp(self):
        self.config = PositionConfig(.266, 53.5)

    def control(self, target=(0, 0, 0), position=(0, 0, 0), velocity=(0, 0, 0),
                attitude=(0, 0, 0), yaw=0, config=None):
        return position_control(config or self.config, position, velocity, attitude, target, yaw)

    def test_level_hover_uses_the_supplied_vehicle_calibration(self):
        for hover in (.266, .5):
            with self.subTest(hover=hover):
                self.assertEqual(self.control(config=replace(self.config, hover_thrust=hover)), BodyRates(thrust=hover))

    def test_horizontal_feedback_has_frd_signs_and_brakes_velocity(self):
        for target, axis, sign in (((1, 0, 0), "pitch_rate", -1), ((0, 1, 0), "roll_rate", 1)):
            with self.subTest(target=target):
                acceleration = getattr(self.control(target=target), axis)
                braking = getattr(self.control(velocity=target), axis)
                self.assertGreater(sign * acceleration, 0)
                self.assertLess(sign * braking, 0)
                reverse = getattr(self.control(target=tuple(-value for value in target)), axis)
                self.assertAlmostEqual(reverse, -acceleration)

    def test_rotating_world_position_and_heading_preserves_body_command(self):
        original = self.control(target=(1, 2, -.2))
        rotated = self.control(target=(-2, 1, -.2), attitude=(0, 0, math.pi / 2), yaw=math.pi / 2)
        self.assertAlmostEqual(original.roll_rate, rotated.roll_rate)
        self.assertAlmostEqual(original.pitch_rate, rotated.pitch_rate)
        self.assertAlmostEqual(original.thrust, rotated.thrust)

    def test_vertical_control_and_tilt_compensation(self):
        self.assertGreater(self.control(target=(0, 0, -1)).thrust, self.config.hover_thrust)
        self.assertLess(self.control(target=(0, 0, 1)).thrust, self.config.hover_thrust)
        tilted = self.control(attitude=(.2, -.1, 0))
        self.assertGreater(tilted.thrust, self.config.hover_thrust)
        self.assertLess(tilted.roll_rate, 0)
        self.assertGreater(tilted.pitch_rate, 0)

    def test_yaw_crosses_the_wrap_boundary_by_the_short_path(self):
        result = self.control(attitude=(0, 0, math.pi - .02), yaw=-math.pi + .02)
        self.assertAlmostEqual(result.yaw_rate, .04 * self.config.attitude_gain)

    def test_body_rates_produce_the_requested_euler_derivatives_when_tilted(self):
        config = replace(self.config, max_rate=10)
        roll, pitch, yaw = .2, -.1, .3
        result = self.control(attitude=(roll, pitch, yaw), yaw=yaw + .1, config=config)
        p, q, r = result.roll_rate, result.pitch_rate, result.yaw_rate
        roll_dot = p + math.sin(roll) * math.tan(pitch) * q + math.cos(roll) * math.tan(pitch) * r
        pitch_dot = math.cos(roll) * q - math.sin(roll) * r
        yaw_dot = (math.sin(roll) * q + math.cos(roll) * r) / math.cos(pitch)
        self.assertAlmostEqual(roll_dot, -roll * config.attitude_gain)
        self.assertAlmostEqual(pitch_dot, -pitch * config.attitude_gain)
        self.assertAlmostEqual(yaw_dot, .1 * config.attitude_gain)

    def test_commands_remain_bounded_for_large_errors(self):
        for target in ((1000, -1000, -100), (-1000, 1000, 100)):
            result = self.control(target=target, attitude=(.8, .5, -2), yaw=2)
            self.assertLessEqual(math.hypot(result.roll_rate, result.pitch_rate, result.yaw_rate), self.config.max_rate + 1e-12)
            self.assertGreaterEqual(result.thrust, 0)
            self.assertLessEqual(result.thrust, 1)

    def test_rejects_invalid_numeric_inputs_and_inverted_attitude(self):
        for values in (dict(position=(math.nan, 0, 0)), dict(velocity=(math.inf, 0, 0)),
                       dict(attitude=(0, math.nan, 0)), dict(target=(0, 0, math.inf)),
                       dict(yaw=math.nan), dict(position=(0, 0)), dict(attitude=(math.pi, 0, 0))):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.control(**values)

    def test_rejects_invalid_vehicle_calibration_and_limits(self):
        for values in (dict(hover_thrust=0), dict(hover_thrust=1), dict(thrust_acceleration=-1),
                       dict(position_gain=math.nan), dict(max_rate=math.inf),
                       dict(max_acceleration=10), dict(max_tilt=math.pi / 2)):
            with self.subTest(values=values):
                config = replace(self.config, **values)
                with self.assertRaises(ValueError):
                    self.control(config=config)

    def test_captured_native_control_steps_replay_without_the_simulator(self):
        fixture = json.loads((Path(__file__).parent / "fixtures/position_control.json").read_text())
        config = PositionConfig(**fixture["config"])
        for case in fixture["cases"]:
            with self.subTest(case=case["name"]):
                actual = position_control(config, **case["inputs"])
                for field, expected in case["expected"].items():
                    self.assertTrue(math.isclose(getattr(actual, field), expected, rel_tol=1e-12, abs_tol=1e-12),
                                    f"{field}: expected {expected}, got {getattr(actual, field)}")


if __name__ == "__main__":
    unittest.main()
