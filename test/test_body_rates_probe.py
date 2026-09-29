from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import unittest

from examples.aigp.probe_body_rates import Probe, RecordedProbe, replay
from miniflight import Attitude, BodyRates, Motion, Ned, PositionNed, State


def state(stamp, down=-3, speed=0):
    return State(stamp, .02, (0, 0, 0), (0, 0, 0), stamp,
                 motion=Motion(stamp, stamp, Ned(0, 0, down), Ned(0, 0, speed)),
                 attitude=Attitude(stamp, stamp, 0, 0, 0))


class ProbeTest(unittest.TestCase):
    def test_command_generator_retains_its_pulses(self):
        command = BodyRates(thrust=.266)
        probe = Probe(command for _ in range(2))
        self.assertEqual(probe.commands, (command, command))
        probe.update(state(0, down=0), 0, None)
        probe.update(state(1), 0, None)
        self.assertEqual(probe.update(state(1.7), 0, None), command)
        with self.assertRaises(ValueError):
            Probe(iter(()))

    def test_pulse_requires_a_settled_hold_and_recovers_before_completion(self):
        command = BodyRates(roll_rate=.25, thrust=.2)
        probe = Probe([command])
        hold = probe.update(state(0, down=0), 0, None)
        self.assertEqual(hold, PositionNed(0, 0, -3))
        self.assertEqual(probe.update(state(1), 0, None), hold)
        self.assertEqual(probe.update(state(1.7), 0, None), command)
        self.assertEqual(probe.update(state(2.2), 0, None), command)
        self.assertEqual(probe.update(state(2.4), 0, None), hold)
        with self.assertRaises(StopIteration):
            probe.update(state(3.1), 0, None)
        self.assertEqual(probe.phase, "done")

    def test_missing_observations_produce_no_command(self):
        probe = Probe([BodyRates(thrust=.2)])
        for missing in ("motion", "attitude"):
            with self.subTest(missing=missing):
                self.assertIsNone(probe.update(replace(state(0), **{missing: None}), 0, None))

    def test_excess_motion_aborts_a_pulse(self):
        probe = Probe([BodyRates(thrust=.2)])
        for sample in (state(0, down=0), state(1), state(1.7)):
            probe.update(sample, 0, None)
        with self.assertRaisesRegex(RuntimeError, "motion envelope"):
            probe.update(state(1.8, speed=6), 0, None)

    def test_a_hold_that_never_settles_has_a_deadline(self):
        probe = Probe([BodyRates(thrust=.2)])
        probe.update(state(0, down=0), 0, None)
        with self.assertRaisesRegex(TimeoutError, "settle"):
            probe.update(state(21, down=0), 0, None)

    def test_zero_rates_precede_position_recovery_and_rotation_must_settle(self):
        pulse, zero = BodyRates(yaw_rate=.25, thrust=.266), BodyRates(thrust=.266)
        probe = Probe([pulse, zero])
        probe.update(state(0, down=0), 0, None)
        probe.update(state(1), 0, None)
        self.assertEqual(probe.update(state(1.7), 0, None), pulse)
        self.assertEqual(probe.update(state(2.4), 0, None), zero)
        self.assertIsInstance(probe.update(state(3.1), 0, None), PositionNed)
        spinning = replace(state(4), gyro=(0, 0, .1))
        self.assertIsInstance(probe.update(spinning, 0, None), PositionNed)
        self.assertIsInstance(probe.update(state(5), 0, None), PositionNed)
        with self.assertRaises(StopIteration):
            probe.update(state(5.7), 0, None)

    def test_recorded_updates_replay_and_detect_changed_commands(self):
        command = BodyRates(pitch_rate=-.25, thrust=.2)
        rows = [{"event": "config", "commands": [asdict(command)], "duration": .6}]
        probe = RecordedProbe([command], .6, lambda **row: rows.append(row))
        for sample in (state(0, down=0), state(1), state(1.7), state(2.4), state(3.1)):
            try:
                probe.update(sample, 0, None)
            except StopIteration:
                pass
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            self.assertEqual(replay(path), {"updates": 5, "completed": True})
            rows[3]["command"]["thrust"] = .9
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            with self.assertRaisesRegex(AssertionError, "replay differs"):
                replay(path)
