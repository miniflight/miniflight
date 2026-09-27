import math
from contextlib import redirect_stderr
from dataclasses import replace
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from target.aigp.simulator import AIGPSimulator, main
from target.aigp.controllers import BaseController, Gate
from target.aigp.controllers.r1_gates import Controller as Gates
from target.aigp.controllers.zero import Controller
from miniflight import BodyRates, Motion, Ned, PositionNed, State, VelocityNed
from target.aigp.simulator import RaceStatus, SimulatorClient


class ControllerConstructionTest(unittest.TestCase):
    def test_base_controller_is_only_an_update_contract(self):
        with self.assertRaises(NotImplementedError):
            BaseController().update(None, 0, None)

    def test_controllers_construct_without_creating_a_client(self):
        with patch.object(SimulatorClient, "__init__", side_effect=AssertionError("controller created a client")):
            Controller()
            Gates()


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(patch("target.aigp.simulator.time.monotonic", side_effect=lambda: self.now))
        self.sleeps = []

        def sleep(seconds):
            self.sleeps.append(seconds)
            self.now += seconds

        self.enterContext(patch("target.aigp.simulator.time.sleep", side_effect=sleep))
        self.sim = Mock(spec=SimulatorClient)
        self.sim.commands = SimulatorClient.commands
        self.sim.gates = None
        self.sim.race_status = RaceStatus(1000, 0, -1, 0, 0, self.now)
        self.state = State(1, .02, (0, 0, 0), (0, 0, 0), self.now)
        self.sim.read.side_effect = [self.state, KeyboardInterrupt()]
        self.controller = SimpleNamespace()
        self.control = BodyRates(thrust=.3)
        self.controller.update = Mock(return_value=self.control)

    def test_zero_is_a_complete_controller(self):
        self.assertEqual(Controller().update(self.state, 0, None), BodyRates())

    def test_simulator_omits_stale_optional_observations_without_mutating_the_snapshot(self):
        motion = Motion(1, self.now - 2, Ned(1, 2, 3), Ned(0, 0, 0))
        state = replace(self.state, motion=motion)
        simulator = AIGPSimulator(self.controller, client=self.sim)
        simulator.control_step(state, 2)
        self.controller.update.assert_called_once_with(replace(state, motion=None), 2, None)
        self.assertIs(state.motion, motion)

    def test_position_controller_stops_when_its_position_sample_becomes_stale(self):
        motion = Motion(1, self.now, Ned(0, 0, 0), Ned(0, 0, 0))
        state = replace(self.state, motion=motion)
        self.sim.gates = (Gate(0, Ned(-10, 0, -2), (1, 0, 0, 0), 2, 2),)
        simulator = AIGPSimulator(Gates(), client=self.sim)
        self.assertIsInstance(simulator.control_step(state, 0), PositionNed)
        self.now += 2
        with self.assertRaisesRegex(ValueError, "fresh VQ1 position"):
            simulator.control_step(replace(state, received_at=self.now), 0)

    def test_runner_creates_its_client_when_none_is_supplied(self):
        with patch("target.aigp.simulator.SimulatorClient", return_value=self.sim) as create:
            with self.assertRaises(KeyboardInterrupt):
                AIGPSimulator(self.controller).rollout(attach=True)
        create.assert_called_once_with()
        self.sim.connect.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_normal_start_and_interrupt_shutdown(self):
        with self.assertRaises(KeyboardInterrupt):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.assertEqual(self.sim.method_calls, [
            call.connect(), call.read(timeout=0.1), call.arm(), call.send(self.control),
            call.heartbeat(), call.read(timeout=0.1), call.send(BodyRates()),
            call.disarm(), call.disconnect(),
        ])
        self.controller.update.assert_called_once_with(self.state, 0, None)
        self.assertAlmostEqual(self.sleeps[0], .02)

    def test_connection_failure_does_not_arm_or_send(self):
        self.sim.connect.side_effect = TimeoutError("no heartbeat")
        with self.assertRaises(TimeoutError):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
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
                    AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
                self.assertEqual(self.sim.send.call_args_list, [call(command), call(BodyRates())])
                self.sim.disarm.assert_called_once()

    def test_unsupported_command_does_not_arm(self):
        self.sim.commands = frozenset((BodyRates,))
        self.controller.update.return_value = PositionNed(1, 2, -3)
        with self.assertRaisesRegex(NotImplementedError, "PositionNed"):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_startup_wait_does_not_arm(self):
        self.controller.update.side_effect = [None, self.control]
        self.sim.read.side_effect = [self.state, self.state, KeyboardInterrupt()]
        with self.assertRaises(KeyboardInterrupt):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.assertEqual(self.sim.method_calls[:5], [
            call.connect(), call.read(timeout=0.1), call.heartbeat(),
            call.read(timeout=0.1), call.arm(),
        ])
        self.sim.arm.assert_called_once()

    def test_completion_disarms_and_closes(self):
        self.controller.update.side_effect = [self.control, StopIteration()]
        self.sim.read.side_effect = [self.state, self.state]
        AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.assertEqual(self.sim.send.call_args, call(BodyRates()))
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_completion_before_first_command_does_not_arm(self):
        self.controller.update.side_effect = StopIteration()
        AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.arm.assert_not_called()
        self.sim.disarm.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_missing_command_after_start_stops_control(self):
        self.controller.update.side_effect = [self.control, None]
        self.sim.read.side_effect = [self.state, self.state]
        with self.assertRaisesRegex(ValueError, "no command"):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.assertEqual(self.sim.send.call_args, call(BodyRates()))
        self.sim.disarm.assert_called_once()

    def test_startup_wait_is_bounded(self):
        def read(**kwargs):
            self.now += 1
            self.sim.race_status = RaceStatus(int(self.now * 1000), 0, -1, 0, 0, self.now)
            return replace(self.state, received_at=self.now)

        self.sim.read.side_effect = read
        self.controller.update.return_value = None
        with self.assertRaisesRegex(TimeoutError, "startup telemetry"):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_invalid_initial_action_never_arms(self):
        self.controller.update.return_value = (0, 0, 0, 0)
        with self.assertRaisesRegex(TypeError, "expected BodyRates"):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
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
                    self.sim.race_status = RaceStatus(int(self.now * 1000), 0, -1, 0, 0, self.now)
                    raise error

                self.state = replace(self.state, received_at=self.now)
                self.sim.race_status = RaceStatus(int(self.now * 1000), 0, -1, 0, 0, self.now)
                self.sim.read.side_effect = read
                with self.assertRaises(type(error)):
                    AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
                self.assertEqual(self.sim.send.call_args, call(BodyRates()))
                self.sim.disarm.assert_called_once()
                self.sim.disconnect.assert_called_once()

    def test_slow_controller_command_is_not_sent(self):
        def slow(state, gate_index, gates):
            self.now += 2
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaisesRegex(TimeoutError, "stale"):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_late_tick_does_not_catch_up_in_a_burst(self):
        def slow(state, gate_index, gates):
            self.now += .15
            return self.control

        self.controller.update.side_effect = slow
        with self.assertRaises(KeyboardInterrupt):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.assertAlmostEqual(self.sleeps[0], .02)

    def test_disarm_and_close_survive_send_failure(self):
        self.sim.send.side_effect = OSError("socket failed")
        with self.assertRaises(OSError):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_close_survives_unexpected_cleanup_failure(self):
        self.sim.send.side_effect = [None, ValueError("cleanup failed")]
        with self.assertRaises(ValueError):
            AIGPSimulator(self.controller, client=self.sim).rollout(attach=True)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_invalid_rate_does_not_connect(self):
        for hz in (0, -1, math.inf, math.nan):
            with self.subTest(hz=hz), self.assertRaises(ValueError):
                AIGPSimulator(self.controller, hz=hz, client=self.sim).rollout(attach=True)
        self.sim.connect.assert_not_called()


class ControllerSelectionTest(unittest.TestCase):
    def test_loads_aigp_controllers_by_short_name(self):
        for name, target in (("zero", "vq2.r2"), ("r1_gates", "vq1.r1")):
            with self.subTest(name=name), patch("target.aigp.simulator.signal.signal"), \
                    patch("target.aigp.simulator.AIGPSimulator") as simulator:
                main([target, "--controller", name])
                controller = simulator.call_args.args[0]
                self.assertIsInstance(controller, BaseController)
                self.assertEqual(type(controller).__module__, f"target.aigp.controllers.{name}")
                simulator.assert_called_once_with(controller, target, 50.0, startup_timeout=120.0)
                simulator.return_value.rollout.assert_called_once_with(attach=False, simulator_args=[])

    def test_controller_without_base_contract_is_rejected_before_launch(self):
        with patch("target.aigp.simulator.AIGPSimulator") as simulator, \
                patch("target.aigp.simulator.importlib.import_module", return_value=SimpleNamespace(Controller=object)), \
                redirect_stderr(io.StringIO()) as error, self.assertRaises(SystemExit) as raised:
            main(["vq2.r2", "--controller", "zero"])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("Controller must inherit BaseController", error.getvalue())
        simulator.assert_not_called()


if __name__ == "__main__":
    unittest.main()
