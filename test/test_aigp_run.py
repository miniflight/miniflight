from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from target.aigp import runner


class AIGPRunTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name).resolve() / "repo with spaces"
        self.base = self.repo / "target/aigp"
        self.base.mkdir(parents=True)
        self.run = self.base / "run"
        shutil.copyfile(Path(runner.__file__).with_name("run"), self.run)
        code = ('import json, os, sys; '
                'print(json.dumps({"args": sys.argv[1:], "cache": os.environ["UV_CACHE_DIR"], '
                '"python": os.environ["UV_PYTHON_INSTALL_DIR"]})); '
                'sys.exit(int(os.getenv("UV_TEST_EXIT", "0")))')
        self.uv = self.base / "uv"
        self.uv.write_text(f'#!/bin/sh\nexec {shlex.quote(sys.executable)} -c {shlex.quote(code)} "$@"\n')
        self.uv.chmod(0o755)
        self.env = dict(os.environ, PATH=f"{self.base}:/usr/bin:/bin")

    def invoke(self, *args):
        return subprocess.run(["/bin/sh", str(self.run), *args], env=self.env, cwd="/",
                              capture_output=True, text=True, timeout=5)

    def test_one_bootstrap_from_any_directory_preserves_arguments(self):
        args = ["vq2.r2", "--controller", "zero", "--", "-test", "value with spaces"]
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        event = json.loads(result.stdout)
        self.assertEqual(event["args"], ["run", "--no-project", "--python", "3.11", "--with-editable",
                                        f"{self.repo}[aigp]", "python", "-m", "target.aigp.runner", *args])
        self.assertEqual(event["cache"], str(self.base / ".runtime/uv-cache"))
        self.assertEqual(event["python"], str(self.base / ".runtime/uv-python"))

    def test_dependency_failure_is_preserved(self):
        self.env["UV_TEST_EXIT"] = "23"
        self.assertEqual(self.invoke("vq1.r1").returncode, 23)

    def test_help_and_missing_target_do_not_need_uv(self):
        self.uv.unlink()
        for args, code in ((["--help"], 0), ([], 2)):
            with self.subTest(args=args):
                result = self.invoke(*args)
                self.assertEqual(result.returncode, code)
                self.assertIn("usage:", result.stdout)

    def test_missing_uv_fails_without_setup(self):
        self.uv.unlink()
        result = self.invoke("vq1.r1")
        self.assertEqual(result.returncode, 1)
        self.assertIn("install uv", result.stderr)
        self.assertFalse((self.base / ".runtime").exists())


class CommandTest(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(runner.signal, "signal"))
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(redirect_stderr(io.StringIO()))
        self.session = self.enterContext(patch.object(runner, "run_session"))
        self.attach = self.enterContext(patch.object(runner, "run"))
        self.launch = self.enterContext(patch.object(runner, "launch"))
        self.process = self.launch.return_value.__enter__.return_value
        self.process.wait.return_value = 0

    def test_simulator_targets_and_alias(self):
        for target in ("vq1", *runner.TARGETS):
            with self.subTest(target=target):
                self.launch.reset_mock()
                self.assertEqual(runner.main([target, "-test", "value with spaces"]), 0)
                self.launch.assert_called_once_with("vq1.r1" if target == "vq1" else target,
                                                    ["-test", "value with spaces"])
        self.session.assert_not_called()

    def test_simulator_exit_code_is_preserved(self):
        for status, expected in ((23, 23), (-15, 143)):
            self.process.wait.return_value = status
            self.assertEqual(runner.main(["vq2.r2"]), expected)

    def test_controller_options_and_simulator_arguments(self):
        self.assertEqual(runner.main(["vq2.r2", "--controller", "zero", "--hz", "40",
                                      "--startup-timeout", "60", "--", "-ResX=800", "value with spaces"]), 0)
        controller = self.session.call_args.args[0]
        self.session.assert_called_once_with(controller, "vq2.r2", 40, ["-ResX=800", "value with spaces"], 60)
        self.launch.assert_not_called()

    def test_attach_never_launches_or_stops_a_simulator(self):
        self.assertEqual(runner.main(["--attach", "--controller", "zero", "--hz", "40"]), 0)
        controller = self.attach.call_args.args[0]
        self.attach.assert_called_once_with(controller, 40)
        self.session.assert_not_called()
        self.launch.assert_not_called()

    def test_invalid_options_do_not_launch(self):
        for args in ([], ["vq1.r2"], ["unknown"], ["vq1.r1", "--controller"],
                     ["vq1.r1", "--controller", "missing"], ["vq1.r1", "--hz", "50"],
                     ["--attach"], ["vq1.r1", "--attach", "--controller", "zero"],
                     ["--attach", "--controller", "zero", "--startup-timeout", "10"],
                     ["--attach", "--controller", "zero", "--", "-test"]):
            with self.subTest(args=args), self.assertRaises(SystemExit) as raised:
                runner.main(args)
            self.assertEqual(raised.exception.code, 2)
        self.launch.assert_not_called()
        self.session.assert_not_called()
        self.attach.assert_not_called()

    def test_keyboard_interrupt_exits_130(self):
        self.process.wait.side_effect = KeyboardInterrupt
        self.assertEqual(runner.main(["vq1.r1"]), 130)
        self.launch.return_value.__exit__.assert_called_once()


if __name__ == "__main__":
    unittest.main()
