from contextlib import redirect_stdout
import io
import signal
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from target.aigp.controllers.r1_gates import Controller as Gates
from target.aigp.controllers.zero import Controller as Zero
from target.aigp.controllers import BaseController
from miniflight import Control, Race
from target.aigp._runtime.controller_runner import _RaceSignals, _drive, _stop_process, run_session


class RaceSignalsTest(unittest.TestCase):
    def setUp(self):
        self.signals = _RaceSignals()

    def race(self, boot=1000, start=-1, finish=-1, gate=0, received=10):
        return Race(boot, start, finish, gate, 0, received)

    def test_wait_countdown_go_gate_pass_finish(self):
        self.assertEqual(self.signals.update(None, 10), "waiting")
        self.assertEqual(self.signals.update(self.race(), 10), "waiting")
        self.assertEqual(self.signals.update(self.race(start=4000), 10), "countdown")
        self.assertEqual(self.signals.update(self.race(boot=3999, start=4000), 10), "countdown")
        self.assertEqual(self.signals.update(self.race(boot=4000, start=4000), 10), "running")
        self.assertEqual(self.signals.update(self.race(boot=5000, start=4000, gate=1), 10), "running")
        self.assertEqual(self.signals.update(self.race(boot=6000, start=4000, gate=6, finish=123), 10), "finished")

    def test_host_elapsed_time_does_not_turn_countdown_into_go(self):
        race = self.race(boot=1000, start=1100)
        self.assertEqual(self.signals.update(race, 10), "countdown")
        self.assertEqual(self.signals.update(race, 10.9), "countdown")
        with self.assertRaisesRegex(TimeoutError, "stale"):
            self.signals.update(race, 11.1)

    def test_reset_signals_stop_a_running_race(self):
        for reset in (self.race(boot=900, start=500), self.race(start=-1),
                      self.race(start=600), self.race(start=500, gate=0)):
            with self.subTest(reset=reset):
                signals = _RaceSignals()
                signals.update(self.race(start=500, gate=1), 10)
                with self.assertRaisesRegex(RuntimeError, "reset"):
                    signals.update(reset, 10)

    def test_race_loss_and_staleness_stop_control(self):
        self.signals.update(self.race(start=0), 10)
        with self.assertRaisesRegex(TimeoutError, "disappeared"):
            self.signals.update(None, 10)
        with self.assertRaisesRegex(TimeoutError, "stale"):
            self.signals.update(self.race(start=0), 12)


class RaceLoopTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(patch("target.aigp._runtime.controller_runner.time.monotonic", side_effect=lambda: self.now))
        self.enterContext(patch("target.aigp._runtime.controller_runner.time.sleep", side_effect=self.sleep))
        self.sim = self.controller = Mock()
        self.controller.update.return_value = Control(thrust=.3)

    def sleep(self, seconds):
        self.now += seconds

    def state(self, boot=1000, start=-1, finish=-1):
        return SimpleNamespace(race=Race(boot, start, finish, 0, 0, self.now),
                               received_at={"HIGHRES_IMU": self.now})

    def test_no_controller_update_arm_or_command_before_go(self):
        states = iter([self.state(), self.state(start=4000), self.state(3999, 4000),
                       self.state(4000, 4000), self.state(5000, 4000, finish=123)])

        def read(**kwargs):
            state = next(states)
            if state.race.sim_boot_time_ms <= 4000:
                self.sim.arm.assert_not_called()
                self.sim.send.assert_not_called()
                self.controller.update.assert_not_called()
            return state

        self.sim.read.side_effect = read
        _drive(self.controller, 50)
        self.controller.update.assert_called_once()
        self.sim.arm.assert_called_once()
        self.assertEqual(self.sim.send.call_args_list, [call(Control(thrust=.3)), call(Control())])
        self.sim.disarm.assert_called_once()

    def test_countdown_has_heartbeats_but_no_actuation(self):
        states = iter([self.state(start=10000)] * 40)

        def read(**kwargs):
            try:
                return next(states)
            except StopIteration:
                raise KeyboardInterrupt

        self.sim.read.side_effect = read
        with self.assertRaises(KeyboardInterrupt):
            _drive(self.controller, 50)
        self.sim.send.assert_not_called()
        self.sim.arm.assert_not_called()
        self.sim.disarm.assert_not_called()
        self.assertEqual(self.sim.heartbeat.call_count, 2)

    def test_no_go_times_out_without_arming(self):
        self.sim.read.side_effect = [self.state(), self.state()]
        with self.assertRaisesRegex(TimeoutError, "never reported GO"):
            _drive(self.controller, 50, startup_deadline=10.01)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_reset_disarms_instead_of_restarting(self):
        self.sim.read.side_effect = [self.state(start=500), self.state(start=-1)]
        with self.assertRaisesRegex(RuntimeError, "reset"):
            _drive(self.controller, 50)
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()
        self.assertEqual(self.sim.send.call_args, call(Control()))

    def test_stale_race_never_arms_despite_fresh_imu(self):
        state = self.state(start=0)
        self.now = 12
        state.received_at["HIGHRES_IMU"] = self.now
        self.sim.read.return_value = state
        with self.assertRaisesRegex(TimeoutError, "race telemetry is stale"):
            _drive(self.controller, 50)
        self.sim.arm.assert_not_called()
        self.controller.update.assert_not_called()

    def test_finish_without_another_imu_sample_is_not_a_timeout(self):
        reads = 0

        def read(**kwargs):
            nonlocal reads
            reads += 1
            if reads == 1:
                return self.state(start=500)
            self.now += .1
            self.sim.race = self.state(2000, 500, finish=123).race
            raise TimeoutError("timed out waiting for fresh IMU telemetry")

        self.sim.read.side_effect = read
        _drive(self.controller, 50)
        self.controller.update.assert_called_once()
        self.assertEqual(self.sim.send.call_args_list, [call(Control(thrust=.3)), call(Control())])
        self.sim.disarm.assert_called_once()

    def test_imu_pause_does_not_replay_commands_or_stop_heartbeats(self):
        reads = 0

        def read(**kwargs):
            nonlocal reads
            reads += 1
            if reads == 1 or reads == 8:
                return self.state(int(self.now * 1000), 500)
            if reads == 9:
                return self.state(int(self.now * 1000), 500, finish=123)
            self.assertLessEqual(kwargs["timeout"], .1)
            self.now += kwargs["timeout"]
            self.sim.race = self.state(int(self.now * 1000), 500).race
            raise TimeoutError("fresh IMU")

        self.sim.read.side_effect = read
        _drive(self.controller, 50)
        self.assertEqual(self.controller.update.call_count, 2)
        self.assertEqual(self.sim.heartbeat.call_count, 2)
        self.assertEqual(self.sim.send.call_args_list,
                         [call(Control(thrust=.3)), call(Control(thrust=.3)), call(Control())])
        self.sim.disarm.assert_called_once()

    def test_go_without_fresh_imu_never_arms(self):
        def read(**kwargs):
            self.now += kwargs["timeout"]
            self.sim.race = self.state(int(self.now * 1000), 500).race
            raise TimeoutError("fresh IMU")

        self.sim.read.side_effect = read
        with self.assertRaisesRegex(TimeoutError, "fresh IMU.*gate_index=0.*finish_ns=-1"):
            _drive(self.controller, 50)
        self.controller.update.assert_not_called()
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.assertGreaterEqual(self.sim.heartbeat.call_count, 2)

    def test_reset_or_stale_finish_without_imu_is_not_success(self):
        for reset in (True, False):
            with self.subTest(reset=reset):
                self.sim.reset_mock()
                self.sim.read.side_effect = [self.state(start=500), TimeoutError("fresh IMU")]
                self.sim.race = (self.state(start=-1).race if reset
                                 else Race(2000, 500, 123, 6, 0, self.now - 2))
                with self.assertRaisesRegex(RuntimeError if reset else TimeoutError,
                                            "reset" if reset else "stale"):
                    _drive(self.controller, 50)
                self.sim.disarm.assert_called_once()


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(patch("target.aigp._runtime.controller_runner.time.monotonic", side_effect=lambda: self.now))
        self.sim, self.process = Mock(spec=BaseController), Mock(pid=98765)
        self.process.poll.return_value = None
        self.process.wait.return_value = 0
        self.ports = self.enterContext(patch("target.aigp._runtime.controller_runner._check_simulator_ports"))
        self.popen = self.enterContext(patch("target.aigp._runtime.controller_runner.subprocess.Popen", return_value=self.process))
        self.drive = self.enterContext(patch("target.aigp._runtime.controller_runner._drive"))

    def test_owns_one_simulator_and_waits_for_telemetry(self):
        self.sim.read.side_effect = [TimeoutError("loading"), object()]
        run_session(self.sim, "vq2.r2", simulator_args=("-ResX=800", "value with spaces"))
        command = self.popen.call_args.args[0]
        self.assertTrue(command[0].endswith("/_runtime/run_vq2.sh"))
        self.assertEqual(command[1:], ["--mode", "r2", "-ResX=800", "value with spaces"])
        self.assertTrue(self.popen.call_args.kwargs["start_new_session"])
        self.sim.open.assert_called_once()
        self.assertEqual(self.sim.read.call_count, 2)
        self.drive.assert_called_once()
        self.assertEqual(self.drive.call_args.args, (self.sim, 50.0))
        self.assertEqual(self.drive.call_args.kwargs, {"process": self.process, "startup_deadline": 130})
        self.sim.disconnect.assert_called_once()
        self.process.terminate.assert_called_once()
        self.process.wait.assert_called_once_with(timeout=10)

    def test_invalid_target_or_incompatible_controller_does_not_launch(self):
        for target, controller in (("vq1.r2", Zero()), ("vq2.r1", Gates()), ("vq2.r2", Gates())):
            with self.subTest(target=target), self.assertRaises(ValueError):
                run_session(controller, target)
        self.popen.assert_not_called()
        self.sim.open.assert_not_called()

    def test_busy_simulator_or_controller_does_not_launch(self):
        self.ports.side_effect = OSError("existing simulator")
        with self.assertRaises(OSError):
            run_session(self.sim, "vq1.r1")
        self.popen.assert_not_called()
        self.ports.side_effect = None
        self.sim.open.side_effect = OSError("existing controller")
        with self.assertRaises(OSError):
            run_session(self.sim, "vq1.r1")
        self.popen.assert_not_called()

    def test_launch_failure_closes_the_receive_ports(self):
        self.popen.side_effect = OSError("launcher missing")
        with self.assertRaises(OSError):
            run_session(self.sim, "vq1.r1")
        self.sim.disconnect.assert_called_once()
        self.drive.assert_not_called()

    def test_early_process_exit_never_runs_or_arms_controller(self):
        self.process.poll.return_value = 2
        with self.assertRaisesRegex(RuntimeError, "status 2"):
            run_session(self.sim, "vq1.r1")
        self.sim.arm.assert_not_called()
        self.drive.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_startup_timeout_stops_only_the_owned_process(self):
        def loading(**kwargs):
            self.now += .2
            raise TimeoutError("no IMU")

        self.sim.read.side_effect = loading
        with self.assertRaisesRegex(TimeoutError, "produce IMU"):
            run_session(self.sim, "vq1.r1", startup_timeout=.3)
        self.drive.assert_not_called()
        self.process.terminate.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_controller_error_or_interrupt_stops_the_simulator(self):
        for error in (RuntimeError("controller failed"), KeyboardInterrupt()):
            with self.subTest(error=error):
                self.process.reset_mock()
                self.sim.reset_mock()
                self.drive.side_effect = error
                with self.assertRaises(type(error)):
                    run_session(self.sim, "vq1.r1")
                self.process.terminate.assert_called_once()
                self.sim.disconnect.assert_called_once()

    def test_unresponsive_owned_process_group_is_killed(self):
        self.process.wait.side_effect = [subprocess.TimeoutExpired("simulator", 10), 0]
        with patch("target.aigp._runtime.controller_runner.os.killpg") as kill:
            _stop_process(self.process)
        kill.assert_called_once_with(98765, signal.SIGKILL)
        self.assertEqual(self.process.wait.call_args_list, [call(timeout=10), call(timeout=5)])


if __name__ == "__main__":
    unittest.main()
