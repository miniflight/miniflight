from pathlib import Path
import subprocess
import sys
import unittest


class CoreBoundaryTest(unittest.TestCase):
    def test_numerical_core_runs_without_site_packages_or_target_implementations(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run([sys.executable, "-S", "-c", """
import sys
from miniflight import BodyRates, State, Vehicle
from miniflight.position import PositionConfig, position_control

assert position_control(PositionConfig(.266, 53.5), (0, 0, 0), (0, 0, 0),
                        (0, 0, 0), (0, 0, 0), 0) == BodyRates(thrust=.266)
for name in ('numpy', 'cv2', 'pymavlink', 'target'):
    assert name not in sys.modules, name
"""], cwd=root, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_existing_import_paths_preserve_class_identity(self):
        import miniflight
        from miniflight import state, vehicle
        for name in ("Attitude", "Frame", "Motion", "MotorOutputs", "Ned", "State"):
            with self.subTest(name=name):
                self.assertIs(getattr(miniflight, name), getattr(state, name))
                self.assertIs(getattr(vehicle, name), getattr(state, name))


if __name__ == "__main__":
    unittest.main()
