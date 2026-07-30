"""Tests for the Miniflight logical line counter."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("miniflight_size", ROOT / "sz.py")
SIZE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIZE)


class SizeTests(unittest.TestCase):
    def test_counter_ignores_comments_and_docstrings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.py"
            path.write_text('"""module docstring"""\n# comment\nvalue = 1\n')

            lines, density = SIZE.logical_lines(path)

        self.assertEqual(lines, 1)
        self.assertGreater(density, 0.0)

    def test_clean_core_has_a_nonzero_count(self):
        total = sum(SIZE.logical_lines(path)[0] for path in SIZE.python_files([ROOT / "miniflight/core"]))

        self.assertGreater(total, 0)


if __name__ == "__main__":
    unittest.main()
