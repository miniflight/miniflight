from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from miniflight import Attitude, BodyRates, Frame, Motion, MotorOutputs, Ned, State
from target.aigp.controllers import Gate
from target.aigp.controllers.r1_gates import Controller as Gates
from target.aigp.experiments.recording import RecordedClient, RecordedController, command_data, read_metadata, recording, replay
from target.aigp.simulator import AIGPSimulator, SimulatorClient


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.directory = self.enterContext(tempfile.TemporaryDirectory())
        self.path = Path(self.directory) / "trace.jsonl"
        self.state = State(1, .02, (1, 2, 3), (.1, .2, .3), 10,
                           motion=Motion(.9, 9.9, Ned(0, 0, 0), Ned(1, 2, 3)),
                           attitude=Attitude(.95, 9.95, .1, .2, .3),
                           motors=MotorOutputs(.8, 9.8, (.1, .2, .3, .4), 15))
        self.gates = (Gate(0, Ned(-10, 1, -2), (1, 0, 0, 0), 2, 3),
                      Gate(1, Ned(-20, -1, -3), (1, 0, 0, 0), 4, 5))

    def test_gate_controller_replays_missing_and_changing_course_inputs(self):
        replacement = (self.gates[0], replace(self.gates[1], center=Ned(-30, 2, -4)))
        with recording(self.path, {"target": "vq1.r1"}) as record:
            controller = RecordedController(Gates(), record)
            self.assertIsNone(controller.update(self.state, 0, None))
            controller.update(self.state, 0, self.gates)
            controller.update(replace(self.state, time=2), 1, replacement)
            controller.update(replace(self.state, time=3), 2, replacement)
        self.assertEqual(read_metadata(self.path), {"target": "vq1.r1"})
        self.assertEqual(replay(Gates(), self.path), 4)
        rows = [json.loads(line) for line in self.path.read_text().splitlines()]
        rows[3]["gates"][1]["center"][0] = -300
        self.path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        with self.assertRaisesRegex(AssertionError, "update 2"):
            replay(Gates(), self.path)

    def test_images_samples_and_numpy_command_values_replay_losslessly(self):
        pixels = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
        pixels.flags.writeable = False
        state = replace(self.state, frame=Frame(7, 1234567890, 9.7, pixels))

        def update(observed, gate_index, gates):
            self.assertEqual(observed.motion, state.motion)
            self.assertEqual(observed.attitude, state.attitude)
            self.assertEqual(observed.motors, state.motors)
            self.assertEqual(gates, self.gates)
            self.assertEqual((observed.frame.id, observed.frame.time_ns, observed.frame.received_at), (7, 1234567890, 9.7))
            np.testing.assert_array_equal(observed.frame.bgr, pixels)
            self.assertFalse(observed.frame.bgr.flags.writeable)
            return BodyRates(thrust=np.float32(observed.frame.bgr[1, 2, 0] / 255))

        with recording(self.path) as record:
            RecordedController(SimpleNamespace(update=update), record).update(state, 0, self.gates)
        self.assertEqual(replay(SimpleNamespace(update=update), self.path), 1)

    def test_exceptions_are_recorded_and_compared_without_controller_internals(self):
        first = SimpleNamespace(update=Mock(side_effect=[BodyRates(), ValueError("missing pose")]))
        with recording(self.path) as record:
            controller = RecordedController(first, record)
            controller.update(self.state, 0, None)
            with self.assertRaisesRegex(ValueError, "missing pose"):
                controller.update(self.state, 0, None)
        fresh = SimpleNamespace(update=Mock(side_effect=[BodyRates(), ValueError("missing pose")]))
        self.assertEqual(replay(fresh, self.path), 2)
        different = SimpleNamespace(update=Mock(side_effect=[BodyRates(), ValueError("different failure")]))
        with self.assertRaisesRegex(AssertionError, "update 1"):
            replay(different, self.path)

    def test_legacy_probe_traces_do_not_require_phase_or_trial_attributes(self):
        rows = [{"event": "config", "duration": .6},
                {"event": "update", "state": asdict(self.state), "gate_index": 0,
                 "phase": "old probe phase", "trial": 99, "command": command_data(BodyRates())},
                {"event": "update", "state": asdict(self.state), "gate_index": 0, "error": "StopIteration"}]
        self.path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        fresh = SimpleNamespace(update=Mock(side_effect=[BodyRates(), StopIteration()]))
        self.assertEqual(replay(fresh, self.path), 2)
        self.assertTrue(all(call.args[2] is None for call in fresh.update.call_args_list))

    def test_wrapping_preserves_target_restrictions(self):
        controller = RecordedController(Gates(), Mock())
        with self.assertRaisesRegex(ValueError, "does not support"):
            AIGPSimulator(controller, "vq2.r1")

    def test_recording_never_overwrites_and_empty_replay_is_rejected(self):
        with recording(self.path):
            pass
        contents = self.path.read_bytes()
        with self.assertRaises(FileExistsError), recording(self.path):
            pass
        self.assertEqual(self.path.read_bytes(), contents)
        controller = SimpleNamespace(update=Mock())
        with self.assertRaisesRegex(ValueError, "no controller updates"):
            replay(controller, self.path)
        controller.update.assert_not_called()

    def test_sent_event_is_emitted_only_after_a_successful_send(self):
        record = Mock()
        client = RecordedClient(record, camera_port=None)
        command = BodyRates(thrust=.266)
        with patch.object(SimulatorClient, "send", side_effect=OSError("socket failed")):
            with self.assertRaises(OSError):
                client.send(command)
        record.assert_not_called()
        with patch.object(SimulatorClient, "send") as send:
            client.send(command)
        send.assert_called_once_with(command)
        record.assert_called_once_with(event="sent", command=command_data(command))
