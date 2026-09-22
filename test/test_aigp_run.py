from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ZSH = shutil.which("zsh")
RUN = Path(__file__).resolve().parents[1] / "sim/aigp/run"


@unittest.skipUnless(ZSH, "zsh is required")
class AIGPRunTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        (self.base / "_runtime").mkdir()
        self.run = self.base / "run"
        shutil.copyfile(RUN, self.run)
        stub = self.base / "_runtime/run_vq1.sh"
        stub.write_text("#!/bin/zsh\nprint -r -- \"${(j:|:)@}\"\n")
        stub.chmod(0o755)
        control = self.base / "control"
        control.write_text('#!/bin/zsh\nprint -r -- "control:${(j:|:)@}"\n')
        control.chmod(0o755)
        stub = self.base / "_runtime/run_vq2.sh"
        stub.write_text("#!/bin/zsh\nprint -r -- \"vq2:${(j:|:)@}\"\n")
        stub.chmod(0o755)

    def invoke(self, *args):
        return subprocess.run([ZSH, str(self.run), *args], capture_output=True,
                              text=True, timeout=5)

    def test_vq1_aliases_forward_arguments(self):
        for mode in ("vq1", "vq1.r1"):
            with self.subTest(mode=mode):
                result = self.invoke(mode, "-test", "value with spaces")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), "-test|value with spaces")

    def test_unintegrated_modes_do_not_fall_back_to_vq1(self):
        for mode in ("vq1.r2",):
            with self.subTest(mode=mode):
                result = self.invoke(mode)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, "")
                self.assertIn(f"{mode} startup is not integrated", result.stderr)

    def test_vq2_rounds_forward_mode_and_arguments(self):
        for mode in ("r1", "r2"):
            with self.subTest(mode=mode):
                result = self.invoke(f"vq2.{mode}", "-test", "value with spaces")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), f"vq2:--mode|{mode}|-test|value with spaces")

    def test_help(self):
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn(f"usage: {self.run}", result.stdout)

    def test_controller_and_simulator_are_one_command(self):
        result = self.invoke("vq1", "--controller", "r1_gates", "--hz", "40", "-test", "value with spaces")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(),
                         "control:r1_gates|--simulator|vq1.r1|--hz|40|--|-test|value with spaces")

    def test_vq2_controller_forwards_round_and_startup_timeout(self):
        result = self.invoke("vq2.r2", "--controller", "zero", "--startup-timeout", "60")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "control:zero|--simulator|vq2.r2|--startup-timeout|60|--")

    def test_incomplete_controller_options_do_not_launch(self):
        for args in (("--controller",), ("--controller", "--hz", "50"), ("--hz", "50")):
            with self.subTest(args=args):
                result = self.invoke("vq1.r1", *args)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, "")

    def test_invalid_or_missing_mode(self):
        for args in ((), ("unknown",)):
            with self.subTest(args=args):
                result = self.invoke(*args)
                self.assertEqual(result.returncode, 2)
                self.assertIn("usage:", result.stderr)


if __name__ == "__main__":
    unittest.main()
