from contextlib import redirect_stderr, redirect_stdout
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from target.aigp import simulator as aigp


class PythonEntrypointTest(unittest.TestCase):
    def invoke(self, *args):
        # Installation supplies the package imports; the script itself owns execution.
        env = dict(os.environ, PYTHONPATH=str(Path(aigp.__file__).resolve().parents[2]))
        with tempfile.TemporaryDirectory() as directory:
            return subprocess.run([sys.executable, aigp.__file__, *args], cwd=directory,
                                  env=env, capture_output=True, text=True, timeout=10)

    def test_python_script_help_from_outside_the_repository(self):
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--controller", result.stdout)
        self.assertIn("--prepare", result.stdout)

    def test_missing_target_is_a_usage_error(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 2)
        self.assertIn("choose vq1.r1", result.stderr)


class CommandTest(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(aigp.signal, "signal"))
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(redirect_stderr(io.StringIO()))
        self.simulator = self.enterContext(patch.object(aigp, "AIGPSimulator"))
        self.launch = self.enterContext(patch.object(aigp, "launch"))
        self.prepare = self.enterContext(patch.object(aigp, "prepare"))
        self.process = self.launch.return_value.__enter__.return_value
        self.process.wait.return_value = 0

    def test_simulator_targets_and_alias(self):
        for target in ("vq1", *aigp.TARGETS):
            with self.subTest(target=target):
                self.launch.reset_mock()
                self.assertEqual(aigp.main([target, "-test", "value with spaces"]), 0)
                self.launch.assert_called_once_with("vq1.r1" if target == "vq1" else target,
                                                    ["-test", "value with spaces"])
        self.simulator.assert_not_called()

    def test_simulator_exit_code_is_preserved(self):
        for status, expected in ((23, 23), (-15, 143)):
            self.process.wait.return_value = status
            self.assertEqual(aigp.main(["vq2.r2"]), expected)

    def test_controller_options_and_simulator_arguments(self):
        self.assertEqual(aigp.main(["vq2.r2", "--controller", "zero", "--hz", "40",
                                      "--startup-timeout", "60", "--", "-ResX=800", "value with spaces"]), 0)
        controller = self.simulator.call_args.args[0]
        self.simulator.assert_called_once_with(controller, "vq2.r2", 40, startup_timeout=60)
        self.simulator.return_value.rollout.assert_called_once_with(attach=False, simulator_args=["-ResX=800", "value with spaces"])
        self.launch.assert_not_called()

    def test_attach_never_launches_or_stops_a_simulator(self):
        self.assertEqual(aigp.main(["--attach", "--controller", "zero", "--hz", "40"]), 0)
        self.simulator.return_value.rollout.assert_called_once_with(attach=True, simulator_args=[])
        self.launch.assert_not_called()

    def test_prepare_never_launches_or_constructs_a_controller(self):
        self.assertEqual(aigp.main(["--prepare", "vq1"]), 0)
        self.prepare.assert_called_once_with("vq1")
        self.launch.assert_not_called()
        self.simulator.assert_not_called()

    def test_invalid_options_do_not_launch(self):
        for args in ([], ["vq1.r2"], ["unknown"], ["vq1.r1", "--controller"],
                     ["vq1.r1", "--controller", "missing"], ["vq1.r1", "--hz", "50"],
                     ["--attach"], ["vq1.r1", "--attach", "--controller", "zero"],
                     ["--attach", "--controller", "zero", "--startup-timeout", "10"],
                     ["--attach", "--controller", "zero", "--", "-test"],
                     ["--prepare", "vq1", "--controller", "zero"]):
            with self.subTest(args=args), self.assertRaises(SystemExit) as raised:
                aigp.main(args)
            self.assertEqual(raised.exception.code, 2)
        self.launch.assert_not_called()
        self.simulator.assert_not_called()
        self.prepare.assert_not_called()

    def test_keyboard_interrupt_exits_130(self):
        self.process.wait.side_effect = KeyboardInterrupt
        self.assertEqual(aigp.main(["vq1.r1"]), 130)
        self.launch.return_value.__exit__.assert_called_once()


if __name__ == "__main__":
    unittest.main()
