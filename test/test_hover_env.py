"""Tests for the deterministic policy-test hover environment."""

import unittest

from miniflight.program import BodyCommand
from miniflight.sim.dynamics import GRAVITY_MPS2, RigidBodyModel
from miniflight.sim.hover_env import HoverEnvironment, HoverEnvironmentModel


def environment(horizon_steps=100):
    body = RigidBodyModel(0.1, (0.001, 0.001, 0.002))
    model = HoverEnvironmentModel(body, rate_time_constant_s=0.05, max_angular_accel_rad_s2=10.0)
    return HoverEnvironment(model, dt_s=0.01, horizon_steps=horizon_steps)


class HoverEnvironmentTests(unittest.TestCase):
    def test_support_command_holds_level_stationary_origin(self):
        env = environment()
        command = BodyCommand(GRAVITY_MPS2, 0.0, 0.0, 0.0)

        for _ in range(100):
            state, terminal = env.step(command)

        self.assertTrue(terminal)
        self.assertEqual(state.position_world_m, (0.0, 0.0, 0.0))
        self.assertEqual(state.velocity_world_mps, (0.0, 0.0, 0.0))

    def test_rate_command_produces_bounded_roll_acceleration(self):
        env = environment()

        state, terminal = env.step(BodyCommand(GRAVITY_MPS2, 1.0, 0.0, 0.0))

        self.assertFalse(terminal)
        self.assertAlmostEqual(state.body_rates_rad_s[0], 0.1)

    def test_horizon_ends_an_episode(self):
        env = environment(horizon_steps=1)

        _, terminal = env.step(BodyCommand(GRAVITY_MPS2, 0.0, 0.0, 0.0))

        self.assertTrue(terminal)


if __name__ == "__main__":
    unittest.main()
