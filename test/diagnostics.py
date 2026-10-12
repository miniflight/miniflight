from pathlib import Path
import tempfile
import unittest

from miniflight.diagnostics import JsonLines


class DiagnosticsTest(unittest.TestCase):
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
