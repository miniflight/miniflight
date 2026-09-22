import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from sim.aigp._runtime.controller_runner import run
from controllers.zero import Controller
from miniflight import Control, PositionNed, Race


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(patch("sim.aigp._runtime.controller_runner.time.monotonic", side_effect=lambda: self.now))
        self.sleeps = []

        def sleep(seconds):
            self.sleeps.append(seconds)
            self.now += seconds

        self.enterContext(patch("sim.aigp._runtime.controller_runner.time.sleep", side_effect=sleep))
        self.sim = Mock()
        self.state = SimpleNamespace(received_at={"HIGHRES_IMU": self.now},
                                     race=Race(1000, 0, -1, 0, 0, self.now))
        self.sim.read.side_effect = [self.state, KeyboardInterrupt()]
        self.controller = Mock()
        self.control = Control(thrust=.3)
        self.controller.update.return_value = self.control

    def test_zero_is_a_complete_controller(self):
        self.assertEqual(Controller().update(self.state), Control())

    def test_normal_start_and_interrupt_shutdown(self):
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller, self.sim)
        self.assertEqual(self.sim.method_calls, [
            call.connect(), call.read(timeout=0.1), call.arm(), call.send(self.control),
            call.heartbeat(), call.read(timeout=0.1), call.send(Control()),
            call.disarm(), call.disconnect(),
        ])
        self.assertAlmostEqual(self.sleeps[1], .02)

    def test_connection_failure_does_not_arm_or_send(self):
        self.sim.connect.side_effect = TimeoutError("no heartbeat")
        with self.assertRaises(TimeoutError):
            run(self.controller, self.sim)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_position_controller_uses_the_same_loop(self):
        self.controller.update.return_value = PositionNed(1, 2, -3)
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller, self.sim)
        self.assertEqual(self.sim.send.call_args_list, [call(PositionNed(1, 2, -3)), call(Control())])
        self.sim.disarm.assert_called_once()

    def test_startup_wait_does_not_arm(self):
        self.controller.update.side_effect = [None, self.control]
        self.sim.read.side_effect = [self.state, self.state, KeyboardInterrupt()]
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller, self.sim)
        self.assertEqual(self.sim.method_calls[:5], [
            call.connect(), call.read(timeout=0.1), call.heartbeat(),
            call.read(timeout=0.1), call.arm(),
        ])
        self.sim.arm.assert_called_once()

    def test_completion_disarms_and_closes(self):
        self.controller.update.side_effect = [self.control, StopIteration()]
        self.sim.read.side_effect = [self.state, self.state]
        run(self.controller, self.sim)
        self.assertEqual(self.sim.send.call_args, call(Control()))
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_completion_before_first_command_does_not_arm(self):
        self.controller.update.side_effect = StopIteration()
        run(self.controller, self.sim)
        self.sim.arm.assert_not_called()
        self.sim.disarm.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_missing_command_after_start_stops_control(self):
        self.controller.update.side_effect = [self.control, None]
        self.sim.read.side_effect = [self.state, self.state]
        with self.assertRaisesRegex(ValueError, "no command"):
            run(self.controller, self.sim)
        self.assertEqual(self.sim.send.call_args, call(Control()))
        self.sim.disarm.assert_called_once()

    def test_startup_wait_is_bounded(self):
        def read(**kwargs):
            self.now += 1
            return SimpleNamespace(received_at={"HIGHRES_IMU": self.now},
                                   race=Race(int(self.now * 1000), 0, -1, 0, 0, self.now))

        self.sim.read.side_effect = read
        self.controller.update.return_value = None
        with self.assertRaisesRegex(TimeoutError, "startup telemetry"):
            run(self.controller, self.sim)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_invalid_initial_action_never_arms(self):
        self.controller.update.return_value = (0, 0, 0, 0)
        with self.assertRaisesRegex(TypeError, "return Control"):
            run(self.controller, self.sim)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_lost_telemetry_or_controller_failure_stops_control(self):
        for error in (TimeoutError("no IMU"), RuntimeError("controller failed")):
            with self.subTest(error=error):
                self.sim.reset_mock()

                def read(**kwargs):
                    if self.sim.read.call_count == 1:
                        return self.state
                    self.now += kwargs["timeout"]
                    self.sim.race = Race(int(self.now * 1000), 0, -1, 0, 0, self.now)
                    raise error

                self.state.received_at["HIGHRES_IMU"] = self.now
                self.state.race = Race(int(self.now * 1000), 0, -1, 0, 0, self.now)
                self.sim.read.side_effect = read
                with self.assertRaises(type(error)):
                    run(self.controller, self.sim)
                self.assertEqual(self.sim.send.call_args, call(Control()))
                self.sim.disarm.assert_called_once()
                self.sim.disconnect.assert_called_once()

    def test_slow_controller_command_is_not_sent(self):
        def slow(state):
            self.now += 2
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaisesRegex(TimeoutError, "stale"):
            run(self.controller, self.sim)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_late_tick_does_not_catch_up_in_a_burst(self):
        def slow(state):
            self.now += .15
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller, self.sim)
        self.assertAlmostEqual(self.sleeps[1], .02)

    def test_disarm_and_close_survive_send_failure(self):
        self.sim.send.side_effect = OSError("socket failed")
        with self.assertRaises(OSError):
            run(self.controller, self.sim)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_close_survives_unexpected_cleanup_failure(self):
        self.sim.send.side_effect = [None, ValueError("cleanup failed")]
        with self.assertRaises(ValueError):
            run(self.controller, self.sim)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_invalid_rate_does_not_connect(self):
        for hz in (0, -1, math.inf, math.nan):
            with self.subTest(hz=hz), self.assertRaises(ValueError):
                run(self.controller, self.sim, hz)
        self.sim.connect.assert_not_called()


@unittest.skipUnless(shutil.which("zsh"), "zsh is required")
class ControlLauncherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name)
        self.base = self.repo / "sim/aigp"
        self.base.mkdir(parents=True)
        (self.base / "_runtime").mkdir()
        source = Path(__file__).resolve().parents[1] / "sim/aigp/control"
        self.launcher = self.base / "control"
        shutil.copyfile(source, self.launcher)
        (self.base / "_runtime/python.sh").write_text("check_python() { :; }\nprepare_python() { :; }\n")
        interpreter = self.base / ".runtime/client-venv/bin/python"
        interpreter.parent.mkdir(parents=True)
        interpreter.write_text('#!/bin/zsh\nprint -r -- "$PWD"\nprint -r -- "${(j:|:)@}"\n')
        interpreter.chmod(0o755)

    def invoke(self, *args):
        return subprocess.run(["zsh", str(self.launcher), *args], capture_output=True,
                              text=True, cwd="/", timeout=5)

    def test_uses_shared_environment_from_any_directory(self):
        result = self.invoke("mine", "--hz", "40")
        self.assertEqual(result.returncode, 0, result.stderr)
        directory, command = result.stdout.splitlines()
        self.assertEqual(Path(directory).resolve(), self.repo.resolve())
        self.assertEqual(command, "-m|sim.aigp._runtime.controller_runner|mine|--hz|40")

    def test_help_and_missing_controller_do_not_prepare_environment(self):
        (self.base / "_runtime/python.sh").write_text("exit 99\n")
        self.assertEqual(self.invoke("--help").returncode, 0)
        self.assertEqual(self.invoke().returncode, 2)


if __name__ == "__main__":
    unittest.main()
