from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.experiments.probe import analyze_yaw


class YawAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.directory = self.enterContext(tempfile.TemporaryDirectory())
        self.path = Path(self.directory) / "trace.jsonl"
        self.fixture = json.loads((Path(__file__).parent / "fixtures/vq1_yaw_tracking.json").read_text())

    def analyze(self, rows):
        self.path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        return analyze_yaw(self.path)

    def test_captured_packets_preserve_request_gain_and_independent_attitude_check(self):
        for case in self.fixture["cases"]:
            with self.subTest(case=case["name"]):
                step, = self.analyze(case["rows"])["steps"]
                for key in ("request", "gyro_mean", "gain", "wire_yaw_mean", "attitude_yaw_rate"):
                    self.assertAlmostEqual(step[key], case["expected"][key], places=10)
                self.assertAlmostEqual(step["gyro_mean"], step["attitude_yaw_rate"], delta=.01)

    def test_wire_bytes_are_authoritative_over_the_readable_packet_copy(self):
        rows = deepcopy(self.fixture["cases"][0]["rows"])
        expected = self.analyze(rows)
        for row in rows:
            if row["event"] == "wire_sent":
                row["message"]["body_yaw_rate"] = 999
        self.assertEqual(self.analyze(rows), expected)

    def test_failed_or_incomplete_measurements_are_rejected(self):
        rows = deepcopy(self.fixture["cases"][0]["rows"])
        for incomplete in ([], rows[:-1], [dict(row, completed=False) if row["event"] == "result" else row for row in rows]):
            with self.subTest(length=len(incomplete)), self.assertRaisesRegex(ValueError, "completed"):
                self.analyze(incomplete)
        with self.assertRaisesRegex(ValueError, "shorter"):
            self.path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            analyze_yaw(self.path, tail_seconds=10)

    def test_mode_change_mid_step_and_nonfinite_sensor_values_are_rejected(self):
        for mode in ("mask", "gyro"):
            rows = deepcopy(self.fixture["cases"][0]["rows"])
            decoder, encoder = mavlink.MAVLink(None), mavlink.MAVLink(None)
            changed = False
            for row in reversed(rows):
                if "packet_hex" not in row:
                    continue
                message, = decoder.parse_buffer(bytes.fromhex(row["packet_hex"]))
                if mode == "mask" and row["event"] == "wire_sent" and message.body_yaw_rate:
                    message.type_mask = 128
                elif mode == "gyro" and message.get_type() == "HIGHRES_IMU":
                    message.zgyro = float("nan")
                else:
                    continue
                row["packet_hex"] = bytes(message.pack(encoder)).hex()
                changed = True
                break
            self.assertTrue(changed)
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "yaw-only|finite"):
                self.analyze(rows)


if __name__ == "__main__":
    unittest.main()
