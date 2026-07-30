"""Tests for the minimal simulator state contract."""

import unittest

from miniflight.sim.state import RigidBodyState


def state(**changes):
    fields = {
        "timestamp_ns": 0,
        "position_world_m": (0.0, 0.0, 0.0),
        "velocity_world_mps": (0.0, 0.0, 0.0),
        "attitude_body_to_world": (1.0, 0.0, 0.0, 0.0),
        "body_rates_rad_s": (0.0, 0.0, 0.0),
    }
    fields.update(changes)
    return RigidBodyState(**fields)


class RigidBodyStateTests(unittest.TestCase):
    def test_identity_state_is_valid(self):
        self.assertEqual(state().attitude_body_to_world, (1.0, 0.0, 0.0, 0.0))

    def test_non_unit_quaternion_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unit length"):
            state(attitude_body_to_world=(2.0, 0.0, 0.0, 0.0))

    def test_non_finite_state_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            state(position_world_m=(float("nan"), 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
