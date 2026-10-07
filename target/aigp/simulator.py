"""Run a controller against native AI-GP; this host loop does not step physics."""

import argparse
from dataclasses import replace
import importlib
import math
import signal
import subprocess
import sys
import time

from miniflight import BodyRates, Command
from target.aigp.controllers import BaseController
from target.aigp.client import RaceStatus, SimulatorClient
from target.aigp.native import (BASE, BINARIES, SHIPPING, TARGETS, VERSIONS,
                               launch, prepare, sha256)


FINISH_WAIT_SECONDS = 5.0


class AIGPSimulator:
    """Run one controller; own its connection, clock, and race lifecycle."""

    def __init__(self, controller: BaseController[Command], target="vq1.r1", hz=50.0,
                 timeout=1.0, startup_timeout=120.0, client=None):
        if not math.isfinite(hz) or not 0 < hz < 100:
            raise ValueError("hz must be positive and below 100 (VQ1 specification)")
        if not all(math.isfinite(value) and value > 0 for value in (timeout, startup_timeout)):
            raise ValueError("timeouts must be positive and finite")
        if target not in TARGETS:
            raise ValueError(f"unsupported simulator: {target}")
        if target not in getattr(controller, "targets", TARGETS):
            raise ValueError(f"this controller does not support {target}")
        self.controller = controller
        self.target = target
        self.hz = hz
        self.timeout = timeout
        self.startup_timeout = startup_timeout
        self.client = SimulatorClient() if client is None else client
        self.status = None
        self.armed = False
        self._used = False

    def rollout(self, attach=False, simulator_args=()):
        """Own the connection, optional simulator process, and controller loop."""
        if self._used:
            raise RuntimeError("create a fresh controller and simulator for each rollout")
        self._used = True
        try:
            self.client.open()
            with launch(self.target, simulator_args, attach=attach) as process:
                try:
                    now = time.monotonic()
                    next_tick = next_heartbeat = last_imu_at = now
                    deadline = now + self.startup_timeout
                    phase = "startup"
                    while True:
                        now = time.monotonic()
                        if self.client.connected and now >= next_heartbeat:
                            self.client.heartbeat()
                            next_heartbeat = now + 0.5
                        try:
                            state = self.client.read(timeout=min(self.timeout, 0.1))
                            last_imu_at = state.received_at
                        except TimeoutError:
                            state = None
                        now = time.monotonic()
                        previous, status = self.status, self.client.race_status
                        if status is not None:
                            if phase != "startup" and (status.race_start_boot_time_ms != previous.race_start_boot_time_ms
                                            or status.sim_boot_time_ms < previous.sim_boot_time_ms
                                            or status.active_gate_index < previous.active_gate_index):
                                raise RuntimeError("race reset during control; start a new run")
                            if status.finished:
                                self.status = status
                                print("race: finished", flush=True)
                                return status
                            if phase == "startup" and status.started and now - status.received_at <= 1.0:
                                phase = "control"
                                previous = None
                                print("race: running", flush=True)
                            if phase != "startup" and (previous is None or status.active_gate_index != previous.active_gate_index):
                                print(f"race: gate_index={status.active_gate_index}", flush=True)
                            self.status = status
                        if process is not None and (exit_code := process.poll()) is not None:
                            raise RuntimeError(f"simulator exited with status {exit_code}")
                        if phase != "control" and now >= deadline:
                            raise TimeoutError("race never reported GO before the startup deadline" if phase == "startup" else
                                               f"timed out waiting for fresh IMU telemetry; "
                                               f"no native finish within {FINISH_WAIT_SECONDS:g}s; last race: {status}")
                        if phase == "control" and now - last_imu_at >= self.timeout:
                            self.stop()
                            phase, deadline = "finish", now + FINISH_WAIT_SECONDS
                            print("controller: IMU lost; waiting for native finish", flush=True)
                        elif phase == "control" and state is not None:
                            gates = self.client.gates
                            index = status.active_gate_index
                            if index < 0 or (gates is not None and index > len(gates)):
                                raise ValueError(f"gate index {index} does not belong to the published track")

                            for name in ("motion", "attitude", "motors", "frame"):
                                sample = getattr(state, name)
                                if sample is not None and now - sample.received_at > self.timeout:
                                    state = replace(state, **{name: None})
                            try:
                                command = self.controller.update(state, index, gates)
                            except StopIteration:
                                return None
                            now = time.monotonic()
                            if now - state.received_at > self.timeout:
                                raise TimeoutError("controller returned a command for stale IMU telemetry")
                            if command is None and self.armed:
                                raise ValueError("controller returned no command after starting")
                            if command is None and now >= deadline:
                                raise TimeoutError("controller did not receive its required startup telemetry")
                            if command is not None:
                                if type(command) not in self.client.commands:
                                    raise TypeError("expected BodyRates, PositionNed or VelocityNed")
                                if not self.armed:
                                    self.armed = True
                                    self.client.arm()
                                    print(f"controller: running at {self.hz:g} Hz", flush=True)
                                self.client.send(command)
                        next_tick += 1.0 / self.hz
                        if next_tick <= now:
                            next_tick = now + 1.0 / self.hz
                        time.sleep(max(0.0, next_tick - time.monotonic()))
                finally:
                    self.stop()
        finally:
            self.client.disconnect()

    def stop(self):
        if not self.armed:
            return
        self.armed = False
        try:
            self.client.send(BodyRates())
        finally:
            self.client.disarm()


def main(argv=None):
    names = sorted(p.stem for p in (BASE / "controllers").glob("*.py")
                   if not p.name.startswith("_"))
    parser = argparse.ArgumentParser(prog="simulator.py", allow_abbrev=False,
                                     description="Run an AI-GP simulator and a Python controller.")
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
            controller = importlib.import_module(f"target.aigp.controllers.{args.controller}").Controller()
            if not isinstance(controller, BaseController):
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
