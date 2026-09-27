from contextlib import nullcontext, redirect_stdout
from dataclasses import replace
import io
import signal
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from target.aigp.controllers import Gate
from target.aigp.controllers.r1_gates import Controller as Gates
from target.aigp.controllers.zero import Controller as Zero
from miniflight import BodyRates, Ned, State
from target.aigp.simulator import RaceStatus, SimulatorClient
from target.aigp.simulator import AIGPSimulator, _stop_process


class RaceStatusTest(unittest.TestCase):
    def setUp(self):
        self.simulator = AIGPSimulator(Zero(), client=Mock(spec=SimulatorClient))
        self.enterContext(redirect_stdout(io.StringIO()))

    def race(self, boot=1000, start=-1, finish=-1, gate=0, received=10):
        return RaceStatus(boot, start, finish, gate, 0, received)

    def update(self, status, now):
        self.simulator._update_status(status, now)
        return self.simulator.phase

    def test_wait_countdown_go_gate_pass_finish(self):
        self.assertEqual(self.update(None, 10), "waiting")
        self.assertEqual(self.update(self.race(), 10), "waiting")
        self.assertEqual(self.update(self.race(start=4000), 10), "countdown")
        self.assertEqual(self.update(self.race(boot=3999, start=4000), 10), "countdown")
        self.assertEqual(self.update(self.race(boot=4000, start=4000), 10), "running")
        self.assertEqual(self.update(self.race(boot=5000, start=4000, gate=1), 10), "running")
        self.assertEqual(self.update(self.race(boot=6000, start=4000, gate=6, finish=123), 10), "finished")

    def test_host_elapsed_time_does_not_turn_countdown_into_go(self):
        status = self.race(boot=1000, start=1100)
        self.assertEqual(self.update(status, 10), "countdown")
        self.assertEqual(self.update(status, 10.9), "countdown")
        self.assertEqual(self.update(status, 100), "countdown")

    def test_a_stale_initial_go_waits_for_a_fresh_packet(self):
        status = self.race(start=0)
        self.assertEqual(self.update(status, 12), "waiting")
        self.assertEqual(self.update(replace(status, received_at=12), 12), "running")

    def test_reset_signals_stop_a_running_race(self):
        for reset in (self.race(boot=900, start=500), self.race(start=-1),
                      self.race(start=600), self.race(start=500, gate=0)):
            with self.subTest(reset=reset):
                simulator = AIGPSimulator(Zero(), client=Mock(spec=SimulatorClient))
                simulator._update_status(self.race(start=500, gate=1), 10)
                with self.assertRaisesRegex(RuntimeError, "reset"):
                    simulator._update_status(reset, 10)

    def test_running_is_latched_through_race_packet_gaps(self):
        status = self.race(start=0)
        self.assertEqual(self.update(status, 10), "running")
        self.assertEqual(self.update(None, 20), "running")
        self.assertEqual(self.update(status, 30), "running")
        self.assertEqual(self.update(self.race(boot=2000, start=0, gate=6), 30), "running")

    def test_finish_is_a_terminal_event_not_an_expiring_sample(self):
        self.update(self.race(start=0), 10)
        self.assertEqual(self.update(self.race(boot=2000, start=0, finish=123), 20), "finished")
        self.assertEqual(self.update(None, 30), "finished")
        self.assertEqual(self.update(self.race(start=0), 30), "finished")

    def test_finish_from_a_different_race_is_not_accepted(self):
        self.update(self.race(start=500), 10)
        with self.assertRaisesRegex(RuntimeError, "reset"):
            self.update(self.race(start=600, finish=123), 10)


class RaceLoopTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(patch("target.aigp.simulator.time.monotonic", side_effect=lambda: self.now))
        self.enterContext(patch("target.aigp.simulator.time.sleep", side_effect=self.sleep))
        self.controller = SimpleNamespace()
        self.sim = Mock(spec=SimulatorClient)
        self.sim.commands = SimulatorClient.commands
        self.sim.gates = None
        self.gates = tuple(Gate(i, Ned(-20 * (i + 1), 0, -2), (1, 0, 0, 0), 2, 2) for i in range(6))
        self.simulator = AIGPSimulator(self.controller, client=self.sim)
        self.controller.update = Mock(return_value=BodyRates(thrust=.3))

    def execute(self, process=None, startup_deadline=None):
        if startup_deadline is not None:
            self.simulator.startup_timeout = startup_deadline - self.now
        if process is None:
            return self.simulator.rollout(attach=True)
        with patch("target.aigp.simulator.launch", return_value=nullcontext(process)):
            return self.simulator.rollout()

    def sleep(self, seconds):
        self.now += seconds

    def state(self, boot=1000, start=-1, finish=-1, gate=0):
        return SimpleNamespace(race=RaceStatus(boot, start, finish, gate, 0, self.now),
                               state=State(boot * .001, .02, (0, 0, 0), (0, 0, 0), self.now))

    def reading(self, samples):
        iterator = None if callable(samples) else iter(samples)

        def read(**kwargs):
            sample = samples(**kwargs) if callable(samples) else next(iterator)
            if isinstance(sample, BaseException):
                raise sample
            self.sim.race_status = sample.race
            return sample.state

        self.sim.read.side_effect = read

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

        self.reading(read)
        self.execute()
        self.controller.update.assert_called_once()
        self.sim.arm.assert_called_once()
        self.assertEqual(self.sim.send.call_args_list, [call(BodyRates(thrust=.3)), call(BodyRates())])
        self.sim.disarm.assert_called_once()

    def test_countdown_has_heartbeats_but_no_actuation(self):
        states = iter([self.state(start=10000)] * 40)

        def read(**kwargs):
            try:
                return next(states)
            except StopIteration:
                raise KeyboardInterrupt

        self.reading(read)
        with self.assertRaises(KeyboardInterrupt):
            self.execute()
        self.sim.send.assert_not_called()
        self.sim.arm.assert_not_called()
        self.sim.disarm.assert_not_called()
        self.assertEqual(self.sim.heartbeat.call_count, 2)

    def test_no_go_times_out_without_arming(self):
        self.reading([self.state(), self.state()])
        with self.assertRaisesRegex(TimeoutError, "never reported GO"):
            self.execute(startup_deadline=10.01)
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_countdown_without_imu_uses_the_startup_deadline(self):
        def read(**kwargs):
            self.now += kwargs["timeout"]
            self.sim.race_status = self.state(boot=int(self.now * 1000), start=100000).race
            raise TimeoutError("fresh IMU")

        self.reading(read)
        with self.assertRaisesRegex(TimeoutError, "never reported GO"):
            self.execute(startup_deadline=12)
        self.assertLess(self.now, 12.2)
        self.controller.update.assert_not_called()
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_reset_disarms_instead_of_restarting(self):
        self.reading([self.state(start=500), self.state(start=-1)])
        with self.assertRaisesRegex(RuntimeError, "reset"):
            self.execute()
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()
        self.assertEqual(self.sim.send.call_args, call(BodyRates()))

    def test_gate_progress_times_out_in_simulator_seconds_and_disarms(self):
        self.sim.gates = self.gates
        self.reading([self.state(1000, 500), self.state(46000, 500), self.state(47000, 500)])
        with self.assertRaisesRegex(TimeoutError, "gate 1.*45 simulator seconds"):
            self.execute()
        self.assertEqual(self.sim.send.call_args_list, [call(BodyRates(thrust=.3)),
                                                       call(BodyRates(thrust=.3)), call(BodyRates())])
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_gate_timeout_is_configurable(self):
        self.sim.gates = self.gates
        self.simulator = AIGPSimulator(self.controller, client=self.sim, gate_timeout=2)
        self.reading([self.state(1000, 500), self.state(4000, 500)])
        with self.assertRaisesRegex(TimeoutError, "gate 1.*2 simulator seconds"):
            self.execute()
        self.sim.disarm.assert_called_once()

    def test_each_reported_gate_pass_gets_a_new_deadline(self):
        self.sim.gates = self.gates
        self.reading([self.state(1000, 500), self.state(45000, 500, gate=1),
                      self.state(88000, 500, gate=1), self.state(99000, 500, finish=123, gate=6)])
        self.assertEqual(self.execute().race_finish_time_ns, 123)
        self.assertEqual(self.controller.update.call_count, 3)

    def test_last_gate_waits_for_native_finish_without_a_gate_deadline(self):
        self.sim.gates = self.gates
        self.reading([self.state(1000, 500, gate=5), self.state(47000, 500, gate=6),
                      self.state(100000, 500, finish=123, gate=6)])
        self.assertEqual(self.execute().race_finish_time_ns, 123)
        self.assertEqual(self.controller.update.call_count, 2)

    def test_unavailable_track_does_not_impose_a_gate_deadline(self):
        self.reading([self.state(1000, 500), self.state(47000, 500), self.state(100000, 500, finish=123)])
        self.assertEqual(self.execute().race_finish_time_ns, 123)
        self.assertEqual(self.controller.update.call_count, 2)

    def test_stale_race_never_arms_despite_fresh_imu(self):
        state = self.state(start=0)
        self.now = 12
        state.state = replace(state.state, received_at=self.now)
        self.reading([state])
        with self.assertRaisesRegex(TimeoutError, "never reported GO"):
            self.execute(startup_deadline=self.now)
        self.sim.arm.assert_not_called()
        self.controller.update.assert_not_called()

    def test_race_gap_with_fresh_imu_continues_until_native_finish(self):
        running = self.state(start=500).race
        reads = 0

        def read(**kwargs):
            nonlocal reads
            reads += 1
            sample = self.state(boot=1000 + reads * 100, start=500)
            sample.race = running if reads <= 160 else replace(sample.race, race_finish_time_ns=123)
            return sample

        self.reading(read)
        self.execute()
        self.assertGreater(self.now - running.received_at, 3)
        self.assertEqual(self.controller.update.call_count, 160)
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()

    def test_packet_silence_does_not_mark_a_race_finished(self):
        running = self.state(start=500).race

        def read(**kwargs):
            if self.now > 15:
                raise KeyboardInterrupt
            sample = self.state(boot=int(self.now * 1000), start=500)
            sample.race = running
            return sample

        self.reading(read)
        with self.assertRaises(KeyboardInterrupt):
            self.execute()
        self.assertGreater(self.controller.update.call_count, 200)
        self.sim.disarm.assert_called_once()

    def test_finish_without_another_imu_sample_is_not_a_timeout(self):
        reads = 0

        def read(**kwargs):
            nonlocal reads
            reads += 1
            if reads == 1:
                return self.state(start=500)
            self.now += .1
            self.sim.race_status = self.state(2000, 500, finish=123).race
            raise TimeoutError("timed out waiting for fresh IMU telemetry")

        self.reading(read)
        self.execute()
        self.controller.update.assert_called_once()
        self.assertEqual(self.sim.send.call_args_list, [call(BodyRates(thrust=.3)), call(BodyRates())])
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
            self.sim.race_status = self.state(int(self.now * 1000), 500).race
            raise TimeoutError("fresh IMU")

        self.reading(read)
        self.execute()
        self.assertEqual(self.controller.update.call_count, 2)
        self.assertEqual(self.sim.heartbeat.call_count, 2)
        self.assertEqual(self.sim.send.call_args_list,
                         [call(BodyRates(thrust=.3)), call(BodyRates(thrust=.3)), call(BodyRates())])
        self.sim.disarm.assert_called_once()

    def test_go_without_fresh_imu_never_arms(self):
        def read(**kwargs):
            self.now += kwargs["timeout"]
            self.sim.race_status = self.state(int(self.now * 1000), 500).race
            raise TimeoutError("fresh IMU")

        self.reading(read)
        with self.assertRaisesRegex(TimeoutError, "fresh IMU.*gate_index=0.*finish_ns=-1"):
            self.execute()
        self.controller.update.assert_not_called()
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()
        self.assertGreaterEqual(self.sim.heartbeat.call_count, 2)

    def test_reset_without_imu_is_not_success(self):
        first = self.state(start=500)

        def read(**kwargs):
            if self.sim.read.call_count == 1:
                return first
            self.sim.race_status = self.state(start=-1).race
            raise TimeoutError("fresh IMU")

        self.reading(read)
        with self.assertRaisesRegex(RuntimeError, "reset"):
            self.execute()
        self.sim.disarm.assert_called_once()

    def test_delayed_finish_is_received_after_disarming_for_imu_loss(self):
        self.delayed_finish()

    def test_recovered_imu_does_not_rearm_while_waiting_for_finish(self):
        self.delayed_finish(recover_imu=True)

    def delayed_finish(self, recover_imu=False):
        started = self.now

        def read(**kwargs):
            if self.sim.read.call_count == 1:
                return self.state(start=500)
            self.now += kwargs["timeout"]
            elapsed = self.now - started
            self.sim.race_status = self.state(boot=int(elapsed * 1000) + 1000, start=500,
                                       finish=123 if elapsed > 2.5 else -1).race
            if elapsed > 1.2:
                self.sim.disarm.assert_called_once()
                self.assertEqual(self.controller.update.call_count, 1)
                self.assertEqual(self.sim.send.call_count, 2)  # command, then zero
                if recover_imu:
                    return self.state(boot=int(elapsed * 1000) + 1000, start=500,
                                      finish=123 if elapsed > 2.5 else -1)
            raise TimeoutError("fresh IMU")

        self.reading(read)
        self.execute()
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()
        self.assertGreaterEqual(self.sim.heartbeat.call_count, 4)
        self.assertGreater(self.now - started, 2.5)

    def test_finish_wait_has_a_deadline_and_never_implies_success(self):
        started = self.now

        def read(**kwargs):
            if self.sim.read.call_count == 1:
                return self.state(start=500)
            self.now += kwargs["timeout"]
            raise TimeoutError("fresh IMU")

        self.reading(read)
        with self.assertRaisesRegex(TimeoutError, "no native finish within 5s"):
            self.execute()
        self.assertGreaterEqual(self.now - started, 6)
        self.assertLess(self.now - started, 6.3)
        self.controller.update.assert_called_once()
        self.sim.disarm.assert_called_once()

    def test_interrupt_during_finish_wait_stops_immediately(self):
        self.abort_finish_wait("interrupt", KeyboardInterrupt, "")

    def test_reset_during_finish_wait_is_not_success(self):
        self.abort_finish_wait("reset", RuntimeError, "race reset")

    def test_process_exit_during_finish_wait_is_not_success(self):
        self.abort_finish_wait("exit", RuntimeError, "simulator exited with status 9")

    def abort_finish_wait(self, event, error, message):
        started = self.now
        process = Mock()
        process.poll.return_value = None

        def read(**kwargs):
            if self.sim.read.call_count == 1:
                return self.state(start=500)
            self.now += kwargs["timeout"]
            if self.now - started >= 1.5:
                self.sim.disarm.assert_called_once()
                if event == "interrupt":
                    raise KeyboardInterrupt
                if event == "reset":
                    self.sim.race_status = self.state(start=-1).race
                if event == "exit":
                    process.poll.return_value = 9
            raise TimeoutError("fresh IMU")

        self.reading(read)
        with self.assertRaisesRegex(error, message):
            self.execute(process=process)
        self.assertLess(self.now - started, 1.7)
        self.controller.update.assert_called_once()
        self.sim.arm.assert_called_once()
        self.sim.disarm.assert_called_once()
        self.assertEqual(self.sim.send.call_args_list, [call(BodyRates(thrust=.3)), call(BodyRates())])

    def test_finish_during_a_slow_update_does_not_send_the_late_command(self):
        def update(state, gate_index, gates):
            self.now += 2
            self.sim.race_status = self.state(boot=3000, start=500, finish=123).race
            return BodyRates(thrust=.3)

        self.reading([self.state(start=500)])
        self.controller.update.side_effect = update
        self.execute()
        self.sim.arm.assert_not_called()
        self.sim.send.assert_not_called()

    def test_queued_finish_is_read_before_process_exit_is_reported(self):
        process = Mock()
        process.poll.return_value = 0
        self.reading([self.state(start=500, finish=123)])
        self.execute(process=process)
        self.controller.update.assert_not_called()
        self.sim.arm.assert_not_called()

    def test_process_exit_without_finish_is_a_failure(self):
        process = Mock()
        process.poll.side_effect = [None, 9]
        self.reading([self.state(start=500), self.state(start=500)])
        with self.assertRaisesRegex(RuntimeError, "simulator exited with status 9"):
            self.execute(process=process)
        self.controller.update.assert_called_once()
        self.sim.disarm.assert_called_once()


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.enterContext(redirect_stdout(io.StringIO()))
        self.enterContext(patch("target.aigp.simulator.time.monotonic", side_effect=lambda: self.now))
        self.enterContext(patch("target.aigp.simulator.time.sleep", side_effect=self.sleep))
        self.controller = SimpleNamespace(update=Mock(return_value=BodyRates(thrust=.3)))
        self.sim = Mock(spec=SimulatorClient)
        self.sim.commands = SimulatorClient.commands
        self.sim.gates = None
        self.sim.race_status = None
        self.sim.read.side_effect = self.read
        self.process = Mock(pid=98765)
        self.process.poll.return_value = None
        self.process.wait.return_value = 0
        self.launch = self.enterContext(patch("target.aigp.simulator.launch"))
        self.owned = self.launch.return_value
        self.owned.__enter__.return_value = self.process

    def sleep(self, seconds):
        self.now += seconds

    def read(self, **kwargs):
        finished = self.sim.read.call_count > 1
        self.sim.race_status = RaceStatus(int(self.now * 1000), 0, 123 if finished else -1,
                                          6 if finished else 0, 0, self.now)
        return State(self.now, .02, (0, 0, 0), (0, 0, 0), self.now)

    def test_owns_one_simulator_and_waits_for_telemetry(self):
        def enter():
            self.sim.open.assert_called_once()
            self.sim.read.assert_not_called()
            return self.process
        self.owned.__enter__.side_effect = enter
        result = AIGPSimulator(self.controller, "vq2.r2", client=self.sim).rollout(
            simulator_args=("-ResX=800", "value with spaces"))
        self.launch.assert_called_once_with("vq2.r2", ("-ResX=800", "value with spaces"))
        self.assertEqual(self.sim.read.call_count, 2)
        self.controller.update.assert_called_once()
        self.assertEqual(result.active_gate_index, 6)
        self.assertEqual(result.race_finish_time_ns, 123)
        self.sim.disarm.assert_called_once()
        self.sim.disconnect.assert_called_once()
        self.owned.__exit__.assert_called_once_with(None, None, None)

    def test_invalid_target_or_incompatible_controller_does_not_launch(self):
        for target, controller in (("vq1.r2", Zero()), ("vq2.r1", Gates()), ("vq2.r2", Gates())):
            with self.subTest(target=target), self.assertRaises(ValueError):
                AIGPSimulator(controller, target, client=self.sim).rollout()
        self.launch.assert_not_called()
        self.sim.open.assert_not_called()

    def test_busy_controller_does_not_launch(self):
        self.sim.open.side_effect = OSError("existing controller")
        with self.assertRaises(OSError):
            AIGPSimulator(self.controller, client=self.sim).rollout()
        self.launch.assert_not_called()

    def test_launch_failure_closes_the_receive_ports(self):
        self.owned.__enter__.side_effect = OSError("launcher missing")
        with self.assertRaises(OSError):
            AIGPSimulator(self.controller, client=self.sim).rollout()
        self.sim.disconnect.assert_called_once()
        self.controller.update.assert_not_called()

    def test_early_process_exit_never_runs_or_arms_controller(self):
        self.process.poll.return_value = 2
        with self.assertRaisesRegex(RuntimeError, "status 2"):
            AIGPSimulator(self.controller, client=self.sim).rollout()
        self.sim.arm.assert_not_called()
        self.controller.update.assert_not_called()
        self.sim.disconnect.assert_called_once()

    def test_startup_timeout_stops_only_the_owned_process(self):
        def loading(**kwargs):
            self.now += .2
            raise TimeoutError("no IMU")
        self.sim.read.side_effect = loading
        with self.assertRaisesRegex(TimeoutError, "startup deadline"):
            AIGPSimulator(self.controller, startup_timeout=.3, client=self.sim).rollout()
        self.controller.update.assert_not_called()
        self.owned.__exit__.assert_called_once()
        self.sim.disconnect.assert_called_once()

    def test_controller_error_or_interrupt_stops_the_simulator(self):
        for error in (RuntimeError("controller failed"), KeyboardInterrupt()):
            with self.subTest(error=error):
                self.process.reset_mock()
                self.sim.reset_mock()
                self.owned.reset_mock()
                self.controller.update.side_effect = error
                with self.assertRaises(type(error)):
                    AIGPSimulator(self.controller, client=self.sim).rollout()
                self.owned.__exit__.assert_called_once()
                self.sim.disconnect.assert_called_once()

    def test_unresponsive_owned_process_group_is_killed(self):
        self.process.wait.side_effect = [subprocess.TimeoutExpired("simulator", 10), 0]
        with patch("target.aigp.simulator.os.killpg") as kill:
            _stop_process(self.process)
        kill.assert_called_once_with(98765, signal.SIGKILL)
        self.assertEqual(self.process.wait.call_args_list, [call(timeout=10), call(timeout=5)])


if __name__ == "__main__":
    unittest.main()
