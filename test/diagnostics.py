from pathlib import Path
import tempfile
import unittest

from miniflight.diagnostics import JsonLines, chart_data
from target.aigp.experiments.diagnostics import quantities


class DiagnosticsTest(unittest.TestCase):
    def test_quantities_preserve_units_and_missing_channels_remain_missing(self):
        signals = dict(reference_time_s=2, position_ref_ned_m=(4, 6, 3), velocity_ref_ned_m_s=(6, 8, 0),
                       acceleration_request_ned_m_s2=(3, 4, 0), rate_ref_frd_rad_s=(0, 1, 0), gyro_frd_rad_s=(0, .8, 0))
        sample = quantities(signals, (1, 2, 3), (3, 4, 0), (.2, .3, .4, .5), .01, .002)
        self.assertEqual((sample["speed_ref_m_s"], sample["speed_m_s"]), (10, 5))
        self.assertEqual((sample["north_error_m"], sample["east_error_m"], sample["down_error_m"]), (3, 4, 0))
        chart, = chart_data([sample], [("Speed", "m/s", [("Observed", "speed_m_s"), ("Unavailable", "missing")])])
        self.assertEqual(chart["series"][0]["y"], [5])
        self.assertEqual(chart["series"][1]["y"], [None])

    def test_live_records_wait_for_complete_lines_and_are_read_once(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.jsonl"
            reader = JsonLines(path)
            try:
                self.assertEqual(reader.poll(), [])
                path.write_text('{"tick":1')
                self.assertEqual(reader.poll(), [])
                with path.open("a") as source:
                    source.write('}\n{"tick":2}\n')
                self.assertEqual(reader.poll(), [{"tick": 1}, {"tick": 2}])
                self.assertEqual(reader.poll(), [])
            finally:
                reader.close()
