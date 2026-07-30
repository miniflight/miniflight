"""Tests for bench-output safety interlocks."""

import unittest

from miniflight.core.interlock import BenchInterlock
from miniflight.core.state import FlightStateMachine


class BenchInterlockTests(unittest.TestCase):
    def test_all_explicit_requirements_allow_bench_output(self):
        interlock = BenchInterlock()

        allowed = interlock.allows(
            session_state=FlightStateMachine.READY,
            link_live=True,
            operator_enabled=True,
            props_removed=True,
        )

        self.assertTrue(allowed)
        self.assertEqual(interlock.reason, "allowed")

    def test_missing_props_confirmation_rejects_output(self):
        interlock = BenchInterlock()

        allowed = interlock.allows(
            session_state=FlightStateMachine.READY,
            link_live=True,
            operator_enabled=True,
            props_removed=False,
        )

        self.assertFalse(allowed)
        self.assertEqual(interlock.reason, "propellers are not confirmed removed")

    def test_running_state_rejects_bench_output(self):
        interlock = BenchInterlock()

        allowed = interlock.allows(
            session_state=FlightStateMachine.RUNNING,
            link_live=True,
            operator_enabled=True,
            props_removed=True,
        )

        self.assertFalse(allowed)
        self.assertEqual(interlock.reason, "session is not ready")


if __name__ == "__main__":
    unittest.main()
