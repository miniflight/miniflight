"""Run a controller against native AI-GP; this host loop does not step physics."""

import argparse
from contextlib import ExitStack
import importlib
import math
from queue import Empty, Queue
import signal
import subprocess
import sys
from threading import Event, Lock, Thread
import time

from miniflight import BodyRates, Command
from target.aigp.controllers import BaseController
from target.aigp.client import RaceStatus, SimulatorClient
from target.aigp.native import (BASE, BINARIES, SHIPPING, TARGETS, VERSIONS,
                               launch, prepare, sha256)


FINISH_WAIT_SECONDS = 5.0


class AIGPSimulator:
    """Run one controller; own its connection, clock, and race lifecycle."""

    def __init__(self, controller, target="vq1.r1", hz=50.0,
                 timeout=1.0, startup_timeout=120.0, client=None):
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
        self.timeout = timeout
        self.startup_timeout = startup_timeout
        self.client = SimulatorClient() if client is None else client
        self.status = None
        self.armed = False
        self._send_lock = Lock()  # One MAVLink encoder serves control and heartbeat.
        self._used = False

    def rollout(self, attach=False, simulator_args=()):
        """Run once; confirmed race status enables control, IMU loss ends it."""
        if self._used:
            raise RuntimeError("create a fresh controller and simulator for each rollout")
        self._used = True
        with ExitStack() as cleanup:
            cleanup.callback(self.client.disconnect)
            self.client.open()
            process = cleanup.enter_context(launch(self.target, simulator_args, attach=attach))
            arrivals, stopped = Queue(), Event()
            def receive():
                heartbeat_at = 0.0
                try:
                    while not stopped.is_set():
                        packets = self.client.poll(timeout=min(self.timeout, 0.1))
                        arrivals.put((packets, self.client.race_status, self.client.telemetry.get("HIGHRES_IMU")))
                        now = time.monotonic()
                        if self.client.connected and now >= heartbeat_at:
                            with self._send_lock:
                                self.client.heartbeat()
                            heartbeat_at = now + 0.5
                except BaseException as error:
                    arrivals.put(error)
            receiver = Thread(target=receive, name="aigp-rx")
            def stop_receiver():
                stopped.set()
                receiver.join(timeout=2)
                if receiver.is_alive():
                    raise RuntimeError("native receiver did not stop")
            receiver.start()
            cleanup.callback(stop_receiver)
            cleanup.callback(self.stop)
            now = time.monotonic()
            period = 1.0 / self.hz
            next_tick = now - period
            last_imu_at = now
            status = imu = None
            startup_deadline = now + self.startup_timeout
            finish_deadline = None
            while True:
                next_tick += period
                if next_tick < now:
                    next_tick = now + period
                time.sleep(max(0.0, next_tick - time.monotonic()))
                now = time.monotonic()
                packets = []
                while True:
                    try:
                        arrival = arrivals.get_nowait()
                    except Empty:
                        break
                    if isinstance(arrival, BaseException):
                        raise arrival
                    batch, status, imu = arrival
                    packets.extend(batch)
                if imu is not None:
                    last_imu_at = imu._host_received_at
                if self.controller is None and self.client.connected:
                    self.controller = self.create_controller(track=self.client.gates)
                    if not isinstance(self.controller, BaseController) or self.target not in getattr(self.controller, "targets", TARGETS):
                        raise TypeError("create a BaseController that supports the selected target")
                now = time.monotonic()
                previous = self.status
                if status is not None:
                    if previous is not None and (
                            status.race_start_boot_time_ms != previous.race_start_boot_time_ms
                            or status.sim_boot_time_ms < previous.sim_boot_time_ms
                            or status.active_gate_index < previous.active_gate_index):
                        raise RuntimeError("race reset during control; start a new run")
                    if status.finished:
                        self.status = status
                    if previous is not None or (status.started and now - status.received_at <= 1.0):
                        self.status = status
                if (status is None or not status.finished) and process is not None and (exit_code := process.poll()) is not None:
                    raise RuntimeError(f"simulator exited with status {exit_code}")
                if self.status is None:
                    if now >= startup_deadline:
                        raise TimeoutError("race never reported GO before the startup deadline")
                # IMU loss ends control permanently; only native finish can end the wait.
                if self.status is not None and not self.status.finished and finish_deadline is None and now - last_imu_at >= self.timeout:
                    self.stop()
                    finish_deadline = now + FINISH_WAIT_SECONDS
                if finish_deadline is not None and now >= finish_deadline and not self.status.finished:
                    raise TimeoutError(f"no fresh IMU; no native finish after {FINISH_WAIT_SECONDS:g}s; last race: {status}")
                if self.controller is None or (finish_deadline is not None and not self.status.finished):
                    continue
                try:
                    command = self.controller.update(
                        tuple(packet for packet in packets if not isinstance(packet.data, bytes)),
                        tuple(packet.decoded for packet in packets if isinstance(packet.data, bytes) and packet.decoded is not None))
                except StopIteration:
                    return self.status if self.status is not None and self.status.finished else None
                now = time.monotonic()
                if self.status is not None and self.status.finished:
                    return self.status
                if self.status is None:
                    continue
                if self.client.race_status.finished:
                    continue  # Deliver the queued finish packet before accepting another command.
                if now - last_imu_at > self.timeout:
                    raise TimeoutError("controller returned a command for stale IMU telemetry")
                if command is None:
                    if self.armed:
                        raise ValueError("controller returned no command after starting")
                    if now >= startup_deadline:
                        raise TimeoutError("controller did not receive its required startup telemetry")
                    continue
                if type(command) not in self.client.commands:
                    raise TypeError("expected BodyRates, PositionNed or VelocityNed")
                with self._send_lock:
                    if not self.armed:
                        self.armed = True
                        self.client.arm()
                    self.client.send(command)

    def stop(self):
        if not self.armed:
            return
        self.armed = False
        with self._send_lock:
            try:
                self.client.send(BodyRates())
            finally:
                self.client.disarm()


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
