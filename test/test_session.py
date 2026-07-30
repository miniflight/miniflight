"""Tests for one-transition-per-tick session supervision."""

import unittest

from miniflight.core.observation import ObservationTick
from miniflight.core.session import FlightSession
from miniflight.core.state import FlightStateMachine


def tick(sequence, age=0):
    return ObservationTick(
        sequence=sequence,
        timestamp_ns=sequence * 1_000,
        accel_mps2=(0.0, 0.0, -9.81),
        gyro_rad_s=(0.0, 0.0, 0.0),
        source_age_ns=age,
    )


class FlightSessionTests(unittest.TestCase):
    def setUp(self):
        self.session = FlightSession(max_imu_age_ns=100, max_camera_age_ns=100)

    def test_ready_path_advances_one_state_per_tick(self):
        self.session.step(tick(1), link_live=True)
        self.assertEqual(self.session.machine.state, FlightStateMachine.OBSERVING)

        self.session.step(tick(2), link_live=True)
        self.assertEqual(self.session.machine.state, FlightStateMachine.QUALIFYING)

    def test_stale_imu_returns_qualifying_to_observing(self):
        self.session.step(tick(1), link_live=True)
        self.session.step(tick(2), link_live=True)
        self.session.step(tick(3, age=101), link_live=True)

        self.assertEqual(self.session.machine.state, FlightStateMachine.OBSERVING)
        self.assertEqual(self.session.machine.reason, "IMU is stale")

    def test_link_loss_enters_failsafe_after_qualification(self):
        self.session.step(tick(1), link_live=True)
        self.session.step(tick(2), link_live=True)
        self.session.step(tick(3), link_live=False)

        self.assertEqual(self.session.machine.state, FlightStateMachine.FAILSAFE)
        self.assertEqual(self.session.machine.reason, "link lost")

    def test_reused_tick_is_rejected_without_transition(self):
        self.session.step(tick(1), link_live=True)

        with self.assertRaisesRegex(RuntimeError, "sequence must increase"):
            self.session.step(tick(1), link_live=False)

        self.assertEqual(self.session.machine.state, FlightStateMachine.OBSERVING)


if __name__ == "__main__":
    unittest.main()
