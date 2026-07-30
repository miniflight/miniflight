"""Tests for the lockstep flight lifecycle."""

import unittest

from miniflight.core.state import FlightStateMachine


class FlightStateMachineTests(unittest.TestCase):
    def test_new_session_starts_offline(self):
        machine = FlightStateMachine()

        self.assertEqual(machine.state, FlightStateMachine.OFFLINE)
        self.assertEqual(machine.reason, "new session")

    def test_valid_path_to_ready_is_explicit(self):
        machine = FlightStateMachine()

        machine.transition(FlightStateMachine.OBSERVING, "USB link is live")
        machine.transition(FlightStateMachine.QUALIFYING, "samples are fresh")
        machine.transition(FlightStateMachine.CALIBRATING, "operator approved")
        machine.transition(FlightStateMachine.READY, "calibration passed")

        self.assertEqual(machine.state, FlightStateMachine.READY)
        self.assertEqual(machine.reason, "calibration passed")

    def test_invalid_transition_keeps_current_state(self):
        machine = FlightStateMachine()

        with self.assertRaisesRegex(RuntimeError, "offline to running"):
            machine.transition(FlightStateMachine.RUNNING, "skip checks")

        self.assertEqual(machine.state, FlightStateMachine.OFFLINE)
        self.assertEqual(machine.reason, "new session")

    def test_failsafe_returns_only_to_safe_states(self):
        machine = FlightStateMachine()
        machine.transition(FlightStateMachine.OBSERVING, "USB link is live")
        machine.transition(FlightStateMachine.QUALIFYING, "samples are fresh")
        machine.transition(FlightStateMachine.FAILSAFE, "sensor timeout")

        with self.assertRaisesRegex(RuntimeError, "failsafe to running"):
            machine.transition(FlightStateMachine.RUNNING, "unsafe retry")

        machine.transition(FlightStateMachine.OFFLINE, "operator reset")
        self.assertEqual(machine.state, FlightStateMachine.OFFLINE)


if __name__ == "__main__":
    unittest.main()
