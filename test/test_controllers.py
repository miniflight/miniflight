import math
from contextlib import redirect_stderr
from dataclasses import replace
import io
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from target.aigp._runtime.controller_runner import main, run
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import Controller as Gates
from target.aigp.controllers.zero import Controller
from miniflight import BodyRates, PositionNed, State, Vehicle, VelocityNed
from target.aigp import Race, SimulatorClient


class BaseControllerTest(unittest.TestCase):
    def test_construction_is_passive_and_defaults_to_aigp_ports(self):
        with patch.object(SimulatorClient, "_bind") as bind:
            controller = BaseController()
        client = controller.client
        self.assertEqual((client.port, client.camera_port), (14550, 5600))
        self.assertIs(controller.vehicle._target, client)
        self.assertIsNone(controller.vehicle.state)
        self.assertFalse(client.connected)
        self.assertIsNone(client._socket)
        self.assertIsNone(client._vision)
        bind.assert_not_called()

    def test_defaults_inherit_the_port_wrapper(self):
        for kind in (Controller, Gates):
            with self.subTest(kind=kind):
                controller = kind(port=0, camera_port=None)
                self.assertIsInstance(controller, BaseController)
                self.assertEqual((controller.client.port, controller.client.camera_port), (0, None))

    def test_owns_one_pair_of_receive_sockets(self):
        wire, camera = Mock(), Mock()
        controller = Controller(port=14600, camera_port=5700)
        with patch.object(SimulatorClient, "_bind", side_effect=[wire, camera]) as bind:
            controller.client.open()
            try:
                bind.assert_has_calls([call(14600), call(5700)])
                self.assertEqual(bind.call_count, 2)
                with self.assertRaisesRegex(RuntimeError, "already open"):
                    controller.client.open()
            finally:
                controller.vehicle.disconnect()
        wire.close.assert_called_once()
        camera.close.assert_called_once()
        self.assertIsNone(controller.client._socket)
        self.assertIsNone(controller.client._vision)

    def test_update_must_be_implemented(self):
        with self.assertRaises(NotImplementedError):
            BaseController().update(None)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(patch("target.aigp._runtime.controller_runner.time.monotonic", side_effect=lambda: self.now))
        self.sleeps = []

        def sleep(seconds):
            self.sleeps.append(seconds)
            self.now += seconds

        self.enterContext(patch("target.aigp._runtime.controller_runner.time.sleep", side_effect=sleep))
        self.sim = Mock(spec=SimulatorClient)
        self.sim.commands = SimulatorClient.commands
        self.sim.race = Race(1000, 0, -1, 0, 0, self.now)
        self.state = State(1, .02, (0, 0, 0), (0, 0, 0), self.now)
        self.sim.read.side_effect = [self.state, KeyboardInterrupt()]
        self.controller = BaseController()
        self.controller.client = self.sim
        self.controller.vehicle = Vehicle(self.sim)
        self.control = BodyRates(thrust=.3)
        self.controller.update = Mock(return_value=self.control)

    def test_zero_is_a_complete_controller(self):
        self.assertEqual(Controller().update(self.state), BodyRates())

    def test_normal_start_and_interrupt_shutdown(self):
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller)
        self.assertEqual(self.sim.method_calls, [
            call.connect(), call.read(timeout=0.1), call.arm(), call.send(self.control),
            call.heartbeat(), call.read(timeout=0.1), call.send(BodyRates()),
            call.disarm(), call.disconnect(),
        ])
        self.controller.update.assert_called_once_with(self.state)
        self.assertAlmostEqual(self.sleeps[1], .02)

    def test_connection_failure_does_not_arm_or_send(self):
        self.sim.connect.side_effect = TimeoutError("no heartbeat")
        with self.assertRaises(TimeoutError):
            run(self.controller)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_position_and_velocity_controllers_use_the_same_loop(self):
        for command in (PositionNed(1, 2, -3), VelocityNed(1, 2, -3)):
            with self.subTest(command=command):
                self.sim.reset_mock()
                self.sim.read.side_effect = [self.state, KeyboardInterrupt()]
                self.controller.update.return_value = command
                with self.assertRaises(KeyboardInterrupt):
                    run(self.controller)
                self.assertEqual(self.sim.send.call_args_list, [call(command), call(BodyRates())])
                self.sim.disarm.assert_called_once()

    def test_unsupported_command_does_not_arm(self):
        self.sim.commands = frozenset((BodyRates,))
        self.controller.update.return_value = PositionNed(1, 2, -3)
        with self.assertRaisesRegex(NotImplementedError, "PositionNed"):
            run(self.controller)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_startup_wait_does_not_arm(self):
        self.controller.update.side_effect = [None, self.control]
        self.sim.read.side_effect = [self.state, self.state, KeyboardInterrupt()]
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller)
        self.assertEqual(self.sim.method_calls[:5], [
            call.connect(), call.read(timeout=0.1), call.heartbeat(),
            call.read(timeout=0.1), call.arm(),
        ])
        self.sim.arm.assert_called_once()

    def test_completion_disarms_and_closes(self):
        self.controller.update.side_effect = [self.control, StopIteration()]
        self.sim.read.side_effect = [self.state, self.state]
        run(self.controller)
        self.assertEqual(self.sim.send.call_args, call(BodyRates()))
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_completion_before_first_command_does_not_arm(self):
        self.controller.update.side_effect = StopIteration()
        run(self.controller)
        self.sim.arm.assert_not_called()
        self.sim.disarm.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_missing_command_after_start_stops_control(self):
        self.controller.update.side_effect = [self.control, None]
        self.sim.read.side_effect = [self.state, self.state]
        with self.assertRaisesRegex(ValueError, "no command"):
            run(self.controller)
        self.assertEqual(self.sim.send.call_args, call(BodyRates()))
        self.sim.disarm.assert_called_once()

    def test_startup_wait_is_bounded(self):
        def read(**kwargs):
            self.now += 1
            self.sim.race = Race(int(self.now * 1000), 0, -1, 0, 0, self.now)
            return replace(self.state, received_at=self.now)

        self.sim.read.side_effect = read
        self.controller.update.return_value = None
        with self.assertRaisesRegex(TimeoutError, "startup telemetry"):
            run(self.controller)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_invalid_initial_action_never_arms(self):
        self.controller.update.return_value = (0, 0, 0, 0)
        with self.assertRaisesRegex(TypeError, "expected BodyRates"):
            run(self.controller)
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

                self.state = replace(self.state, received_at=self.now)
                self.sim.race = Race(int(self.now * 1000), 0, -1, 0, 0, self.now)
                self.sim.read.side_effect = read
                with self.assertRaises(type(error)):
                    run(self.controller)
                self.assertEqual(self.sim.send.call_args, call(BodyRates()))
                self.sim.disarm.assert_called_once()
                self.sim.disconnect.assert_called_once()

    def test_slow_controller_command_is_not_sent(self):
        def slow(state):
            self.now += 2
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaisesRegex(TimeoutError, "stale"):
            run(self.controller)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_late_tick_does_not_catch_up_in_a_burst(self):
        def slow(state):
            self.now += .15
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaises(KeyboardInterrupt):
            run(self.controller)
        self.assertAlmostEqual(self.sleeps[1], .02)

    def test_disarm_and_close_survive_send_failure(self):
        self.sim.send.side_effect = OSError("socket failed")
        with self.assertRaises(OSError):
            run(self.controller)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_close_survives_unexpected_cleanup_failure(self):
        self.sim.send.side_effect = [None, ValueError("cleanup failed")]
        with self.assertRaises(ValueError):
            run(self.controller)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_invalid_rate_does_not_connect(self):
        for hz in (0, -1, math.inf, math.nan):
            with self.subTest(hz=hz), self.assertRaises(ValueError):
                run(self.controller, hz)
        self.sim.connect.assert_not_called()


class ControllerSelectionTest(unittest.TestCase):
    def test_loads_aigp_controllers_by_short_name(self):
        for name, simulator in (("zero", "vq2.r2"), ("r1_gates", "vq1.r1")):
            with self.subTest(name=name), \
                    patch("target.aigp._runtime.controller_runner.signal.signal"), \
                    patch("target.aigp._runtime.controller_runner.run_session") as session:
                main([name, "--simulator", simulator])
                controller = session.call_args.args[0]
                self.assertEqual(type(controller).__module__, f"target.aigp.controllers.{name}")
                session.assert_called_once_with(controller, simulator, 50.0, [], 120.0)

    def test_plain_controller_is_rejected_before_launch(self):
        with patch("target.aigp._runtime.controller_runner.run_session") as session, \
                patch("target.aigp._runtime.controller_runner.importlib.import_module",
                      return_value=SimpleNamespace(Controller=object)), \
                redirect_stderr(io.StringIO()) as error, self.assertRaises(SystemExit) as raised:
            main(["zero", "--simulator", "vq2.r2"])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("Controller must inherit BaseController", error.getvalue())
        session.assert_not_called()


@unittest.skipUnless(shutil.which("zsh"), "zsh is required")
class ControlLauncherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name)
        self.base = self.repo / "target/aigp"
        self.base.mkdir(parents=True)
        (self.base / "_runtime").mkdir()
        source = Path(__file__).resolve().parents[1] / "target/aigp/control"
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
        self.assertEqual(command, "-m|target.aigp._runtime.controller_runner|mine|--hz|40")

    def test_help_and_missing_controller_do_not_prepare_environment(self):
        (self.base / "_runtime/python.sh").write_text("exit 99\n")
        self.assertEqual(self.invoke("--help").returncode, 0)
        self.assertEqual(self.invoke().returncode, 2)


if __name__ == "__main__":
    unittest.main()
