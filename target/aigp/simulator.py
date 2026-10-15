"""Run a controller against native AI-GP; this host loop does not step physics."""

import argparse
from contextlib import ExitStack
from dataclasses import asdict, replace
import importlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

from miniflight import BodyRates, Motion, Ned, vehicle
from miniflight.loop import run
from miniflight.monitor import Monitor
from target.aigp.controllers import BaseController
from target.aigp.client import SimulatorClient
from target.aigp.native import BASE, TARGETS, VERSIONS, RaceStateReader, launch, prepare


FINISH_WAIT_SECONDS = 5.0


class AIGPSimulator:
    """Run one controller; own its connection, clock, and race lifecycle."""

    def __init__(self, controller, target="vq1.r1", hz=50.0,
                 timeout=1.0, startup_timeout=120.0, client=None, race_status_path=None, monitor_path=None, race_reader=None, clock=time):
        if not math.isfinite(hz) or not 0 < hz < 100:
            raise ValueError("hz must be positive and below 100 (VQ1 specification)")
        if not all(math.isfinite(value) and value > 0 for value in (timeout, startup_timeout)):
            raise ValueError("timeouts must be positive and finite")
        if target not in TARGETS:
            raise ValueError(f"unsupported simulator: {target}")
        if target not in getattr(controller, "targets", TARGETS):
            raise ValueError(f"this controller does not support {target}")
        self.create_controller = controller
        self.controller = None
        self.target = target
        self.hz = hz
        self.clock = clock
        self.timeout = timeout
        self.startup_timeout = startup_timeout
        self.client = SimulatorClient() if client is None else client
        self.vehicle = vehicle
        self.monitor = Monitor()
        self.monitor_path = Path(monitor_path) if monitor_path is not None else BASE / ".runtime/rollouts" / f"{target}-{time.time_ns()}.png"
        self.status = None
        self.race_status_path = race_status_path or BASE / ".runtime" / TARGETS[target][0] / "race-status.jsonl"
        self.race_reader = race_reader
        self.arm_requested = False
        self._used = False

    def rollout(self, attach=False, simulator_args=()):
        """Run once; confirmed race status enables control, IMU loss ends it."""
        if self._used:
            raise RuntimeError("create a fresh controller and simulator for each rollout")
        self._used = True
        with ExitStack() as cleanup:
            cleanup.callback(self.save_monitor)  # Plot only after control, receiver, process and connection stop.
            self.vehicle.bind(self.client)
            cleanup.callback(self.vehicle.disconnect)
            self.vehicle.connect()
            process = cleanup.enter_context(launch(self.target, simulator_args, attach=attach,
                                                   race_status_path=self.race_status_path))
            race_reader = self.race_reader if self.race_reader is not None else RaceStateReader(self.race_status_path, self.timeout)
            if self.race_reader is None:
                cleanup.callback(race_reader.close)
            cleanup.callback(self.stop)
            now = self.clock.monotonic()
            heartbeat_at = 0.0
            startup_deadline = now + self.startup_timeout
            finish_deadline = None
            def read_signals(now):
                nonlocal finish_deadline, heartbeat_at
                state = self.vehicle.read(timeout=0)
                packets = self.client.packets
                for packet in packets:
                    message = packet.data
                    if not isinstance(message, bytes) and message.get_type() == "LOCAL_POSITION_NED":
                        self.monitor.observe(Motion(message.time_boot_ms / 1000, packet.received_at,
                                                    Ned(message.x, message.y, message.z),
                                                    Ned(message.vx, message.vy, message.vz)))
                if self.client.connected and now >= heartbeat_at:
                    self.client.heartbeat()
                    heartbeat_at = now + .5
                if self.controller is None and self.client.connected:
                    self.controller = self.create_controller()
                    if not isinstance(self.controller, BaseController) or self.target not in getattr(self.controller, "targets", TARGETS):
                        raise TypeError("create a BaseController that supports the selected target")
                now = self.clock.monotonic()
                self.read_race(race_reader.poll())
                if (self.status is None or not self.status.completed) and process is not None and (exit_code := process.poll()) is not None:
                    raise RuntimeError(f"simulator exited with status {exit_code}")
                if self.status is None or not self.status.started:
                    if now >= startup_deadline:
                        raise TimeoutError("race never reported GO before the startup deadline")
                # IMU loss ends control permanently; only native finish can end the wait.
                if (self.status is not None and self.status.started and not self.status.completed
                        and finish_deadline is None and state is not None and now - state.received_at >= self.timeout):
                    self.stop()
                    finish_deadline = now + FINISH_WAIT_SECONDS
                if finish_deadline is not None and now >= finish_deadline and not self.status.completed:
                    raise TimeoutError(f"no fresh IMU; no native finish after {FINISH_WAIT_SECONDS:g}s; last race: {self.status}")
                if self.controller is None or (finish_deadline is not None and not self.status.completed):
                    return None
                return (tuple(packet for packet in packets if not isinstance(packet.data, bytes)),
                        tuple(packet.decoded for packet in packets if isinstance(packet.data, bytes) and packet.decoded is not None),
                        None if self.status is None else replace(self.status, control_time=self.clock.monotonic()))

            def send_command(command):
                now = self.clock.monotonic()
                state = self.vehicle.state
                self.read_race(race_reader.poll())  # A slow controller cannot hide a new invalidation.
                if self.status is not None and self.status.completed:
                    raise StopIteration
                if self.status is None or not self.status.started:
                    return
                if command is None or state is None:
                    if self.arm_requested:
                        raise ValueError("controller returned no command after starting")
                    if now >= startup_deadline:
                        raise TimeoutError("controller did not receive its required startup telemetry")
                    return
                if now - state.received_at > self.timeout:
                    raise TimeoutError("controller returned a command for stale IMU telemetry")
                if not self.arm_requested:
                    self.vehicle.validate(command)
                    self.arm_requested = True
                    self.vehicle.arm()
                self.vehicle.send(command)

            try:
                run(read_signals, lambda *signals: self.controller.update(*signals), send_command, hz=self.hz, clock=self.clock)
            except StopIteration:
                self.read_race(race_reader.poll())
                return self.status if self.status is not None and self.status.completed else None

    def save_monitor(self):
        self.monitor.save(self.monitor_path.with_suffix(".csv"))
        race = self.status
        with self.monitor_path.with_suffix(".sources.json").open("x") as output:
            json.dump(dict(target=self.target, hz=self.hz, position_samples=len(self.monitor.motion),
                           position_source="MAVLink.LOCAL_POSITION_NED (privileged)",
                           position_frame="local NED metres; velocity in m/s",
                           device_time="LOCAL_POSITION_NED.time_boot_ms / 1000 (seconds)",
                           host_received_at="Packet.received_at (host monotonic seconds)",
                           reference_source="target/aigp/config/race.lua: mesh bounds centers (privileged)",
                           reference_frame="local NED metres relative to starting grid",
                           race_source="target/aigp/config/race.lua: engine observations (privileged)",
                           race_clock="GetRaceTime (game seconds); received_at is host monotonic file-read time",
                           clocks_synchronized=False, race_state=None if race is None else asdict(race)), output, indent=2)
        outcome = "not observed" if race is None else f"completed={race.completed}, valid={race.valid}, gate={race.active_gate_index}"
        self.monitor.plot(self.monitor_path, () if race is None else race.gate_centers,
                          title=f"{self.target}: MAVLink LOCAL_POSITION_NED (privileged)\nNative Lua race: {outcome}",
                          reference_label="Lua mesh bounds centers (privileged)")
        print(f"Rollout plot: {self.monitor_path}", flush=True)

    def read_race(self, arrivals):
        for state in arrivals:
            previous, self.status = self.status, state
            if not state.valid:
                raise RuntimeError(f"native race is invalid: {state}")
            if previous is not None and previous.started and (not state.started or state.active_gate_index < previous.active_gate_index):
                raise RuntimeError("native race reset during control")
        if (self.status is not None and self.status.started
                and self.clock.monotonic() - self.status.received_at > self.timeout):
            raise TimeoutError("native race state is stale")

    def stop(self):
        if not self.arm_requested:
            return
        self.arm_requested = False
        try:
            self.vehicle.send(BodyRates())
        finally:
            self.vehicle.disarm()


def main(argv=None):
    names = sorted(p.stem for p in (BASE / "controllers").glob("*.py")
                   if not p.name.startswith("_"))
    parser = argparse.ArgumentParser(prog="simulator.py", allow_abbrev=False,
                                     description="Run a direct AI-GP arena; training/qualification selection is not implemented.")
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("target", nargs="?", choices=(*TARGETS, "vq1"))
    parser.add_argument("--controller", choices=names)
    operation.add_argument("--prepare", choices=VERSIONS, help="extract and configure a simulator without launching it")
    operation.add_argument("--attach", action="store_true", help="control an already running simulator")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--startup-timeout", type=float)
    argv = list(sys.argv[1:] if argv is None else argv)
    boundary = argv.index("--") if "--" in argv else len(argv)
    args = parser.parse_args(argv[:boundary])
    simulator_args = argv[boundary + 1:]
    if args.prepare:
        if args.controller or simulator_args or args.hz is not None or args.startup_timeout is not None:
            parser.error("--prepare cannot be combined with run options")
        prepare(args.prepare)
        return 0
    if args.target == "vq1":
        args.target = "vq1.r1"
    if args.attach and (simulator_args or args.startup_timeout is not None or not args.controller):
        parser.error("--attach requires --controller and no simulator arguments")
    if not args.controller and (args.hz is not None or args.startup_timeout is not None):
        parser.error("--hz and --startup-timeout require --controller")
    hz = 50.0 if args.hz is None else args.hz
    startup_timeout = 120.0 if args.startup_timeout is None else args.startup_timeout

    def stop(signum, frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    try:
        if args.controller:
            controller = importlib.import_module(f"target.aigp.controllers.{args.controller}").Controller
            if not issubclass(controller, BaseController):
                parser.error("Controller must inherit BaseController")
            sim = AIGPSimulator(controller, args.target or "vq1.r1", hz, startup_timeout=startup_timeout)
            sim.rollout(attach=args.attach, simulator_args=simulator_args)
        else:
            with launch(args.target, simulator_args) as process:
                status = process.wait()
            return status if status >= 0 else 128 - status
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, TypeError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(1, f"{error}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
