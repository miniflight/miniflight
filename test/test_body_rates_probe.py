from dataclasses import asdict, replace
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.experiments.probe import DiagnosticClient, Probe, replay
from miniflight import Attitude, BodyRates, Motion, Ned, PositionNed, State
from target.aigp.experiments.recording import RecordedController, recording


def state(stamp, down=-3, speed=0):
    return State(stamp, .02, (0, 0, 0), (0, 0, 0), stamp,
                 motion=Motion(stamp, stamp, Ned(0, 0, down), Ned(0, 0, speed)),
                 attitude=Attitude(stamp, stamp, 0, 0, 0))


class ProbeTest(unittest.TestCase):
    def test_yaw_example_selects_only_yaw_commands(self):
        from examples.aigp import probe_body_rates as example
        with patch.object(example, "run", return_value={"completed": True}) as run, \
                patch.object(example.signal, "signal"), patch.object(example.signal, "alarm"), \
                redirect_stdout(io.StringIO()):
            example.main(["yaw", "trace.jsonl", "--thrust", ".266", "--duration", "2", "--repeat", "3", "--hz", "20"])
        path, commands, duration = run.call_args.args
        self.assertEqual(path, Path("trace.jsonl"))
        self.assertEqual(duration, 2)
        self.assertEqual(run.call_args.kwargs, {"hz": 20, "yaw_feedback": False})
        self.assertEqual([command.yaw_rate for command in commands], [.25, -.25] * 3)
        self.assertTrue(all(command.roll_rate == command.pitch_rate == 0 for command in commands))

    def test_longer_steps_are_limited_to_yaw_and_include_neutral_recovery(self):
        yaw, zero = BodyRates(yaw_rate=.25, thrust=.266), BodyRates(thrust=.266)
        probe = Probe([yaw, zero], duration=2)
        probe.update(state(0, down=0), 0, None)
        probe.update(state(1), 0, None)
        self.assertEqual(probe.update(state(1.7), 0, None), yaw)
        self.assertEqual(probe.update(state(3.6), 0, None), yaw)
        self.assertEqual(probe.update(state(3.8), 0, None), zero)
        for commands, duration in (([BodyRates(roll_rate=.1)], 2), ([BodyRates(pitch_rate=.1)], 2),
                                   ([zero], 2), ([yaw], 3.1), ([yaw], 0)):
            with self.subTest(commands=commands, duration=duration), self.assertRaises(ValueError):
                Probe(commands, duration)

    def test_diagnostic_records_the_actual_successfully_transmitted_packet(self):
        record = Mock()
        client = DiagnosticClient(record)
        client._socket, client._peer = Mock(), ("127.0.0.1", 14560)
        encoder = mavlink.MAVLink(None)
        message = mavlink.MAVLink_set_attitude_target_message(123, 1, 1, 144, [1, 0, 0, 0], 0, 0, -.25, .266)
        packet = message.pack(encoder)
        client.write(packet)
        client._socket.sendto.assert_called_once_with(packet, client._peer)
        row = record.call_args.kwargs
        self.assertEqual(row["event"], "wire_sent")
        self.assertEqual(bytes.fromhex(row["packet_hex"]), packet)
        self.assertEqual(row["message"]["type_mask"], 144)
        self.assertEqual(row["message"]["body_yaw_rate"], -.25)
        record.reset_mock()
        client._socket.sendto.side_effect = OSError("send failed")
        with self.assertRaises(OSError):
            client.write(packet)
        record.assert_not_called()

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
        self.assertEqual(probe.update(state(2.4), 0, None), BodyRates(thrust=.2))
        self.assertEqual(probe.update(state(3.1), 0, None), hold)
        with self.assertRaises(StopIteration):
            probe.update(state(3.8), 0, None)
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
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.jsonl"
            with recording(path, {"commands": [asdict(command)], "duration": .6}) as record:
                probe = RecordedController(Probe([command]), record)
                for sample in (state(0, down=0), state(1), state(1.7), state(2.4), state(3.1), state(3.8)):
                    try:
                        probe.update(sample, 0, None)
                    except StopIteration:
                        pass
            self.assertEqual(replay(path), {"updates": 6, "completed": True})
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[3]["command"]["thrust"] = .9
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            with self.assertRaisesRegex(AssertionError, "replay differs"):
                replay(path)
