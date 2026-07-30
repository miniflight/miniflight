"""Physics checks for the vacuum rigid-body model."""

import unittest

from miniflight.sim.dynamics import BodyWrench, GRAVITY_MPS2, RigidBodyModel, step
from miniflight.sim.state import RigidBodyState


MODEL = RigidBodyModel(0.1, (0.001, 0.001, 0.002))
STATE = RigidBodyState(0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


class DynamicsTests(unittest.TestCase):
    def test_balanced_force_holds_an_identity_vehicle(self):
        next_state = step(MODEL, STATE, BodyWrench((0.0, 0.0, -MODEL.mass_kg * GRAVITY_MPS2), (0.0, 0.0, 0.0)), 1.0)

        self.assertEqual(next_state.position_world_m, (0.0, 0.0, 0.0))
        self.assertEqual(next_state.velocity_world_mps, (0.0, 0.0, 0.0))

    def test_free_fall_matches_constant_acceleration(self):
        next_state = step(MODEL, STATE, BodyWrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 1.0)

        self.assertAlmostEqual(next_state.position_world_m[2], 0.5 * GRAVITY_MPS2)
        self.assertAlmostEqual(next_state.velocity_world_mps[2], GRAVITY_MPS2)

    def test_roll_torque_changes_roll_rate(self):
        next_state = step(MODEL, STATE, BodyWrench((0.0, 0.0, 0.0), (0.001, 0.0, 0.0)), 0.1)

        self.assertAlmostEqual(next_state.body_rates_rad_s[0], 0.1)

    def test_invalid_step_time_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            step(MODEL, STATE, BodyWrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
