import argparse
from contextlib import suppress
import importlib
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from miniflight import BodyRates
from target.aigp import controllers
from target.aigp.controllers import BaseController


FINISH_WAIT_SECONDS = 5.0


class _RaceSignals:
    """Latch native race events; packet delivery is not a lease on running."""

    def __init__(self):
        self.start = None
        self.previous = None
        self.phase = "waiting"

    def update(self, race, now):
        if self.phase == "finished" or race is None:
            return self.phase
        if self.start is not None:
            if (race.race_start_boot_time_ms != self.start
                    or race.sim_boot_time_ms < self.previous.sim_boot_time_ms
                    or race.active_gate_index < self.previous.active_gate_index):
                raise RuntimeError("race reset during control; start a new run")
        self.previous = race
        if race.race_finish_time_ns >= 0:
            self.phase = "finished"
        elif self.start is not None:
            self.phase = "running"
        elif race.race_start_boot_time_ms < 0:
            self.phase = "waiting"
        elif race.sim_boot_time_ms < race.race_start_boot_time_ms:
            self.phase = "countdown"
        elif now - race.received_at <= 1.0:
            # Only a fresh native GO may begin control. Once observed it stays
            # true until native finish/reset, independently of the sensor clock.
            self.start = race.race_start_boot_time_ms
            self.phase = "running"
        else:
            self.phase = "waiting"
        return self.phase


def run(controller: BaseController, hz=50.0, timeout=1.0):
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("hz must be positive and finite")
    try:
        controller.vehicle.connect()
        _drive(controller, hz, timeout)
    finally:
        controller.vehicle.disconnect()


def _drive(controller: BaseController, hz, timeout=1.0, process=None, startup_deadline=None):
    vehicle, client = controller.vehicle, controller.client
    armed = False
    period = 1.0 / hz
    next_tick = next_heartbeat = time.monotonic()
    last_imu_at = next_tick
    if startup_deadline is None:
        startup_deadline = next_tick + 120
    signals, phase, controller_deadline = _RaceSignals(), None, None
    imu_error, finish_deadline = None, None
    try:
        while True:
            time.sleep(max(0.0, next_tick - time.monotonic()))
            try:
                # Race events and heartbeats must keep moving when IMU stops.
                state = vehicle.read(timeout=min(timeout, 0.1))
                last_imu_at = state.received_at
            except TimeoutError:
                state = None
            now = time.monotonic()
            race = controller.race
            current_phase = signals.update(race, now)
            if current_phase != phase:
                print(f"race: {current_phase}", flush=True)
                phase = current_phase
            if phase == "finished":
                return
            # Drain the socket before checking exit: the last datagram can be
            # the finish signal even if the owned process has just exited.
            _check_process(process)
            if phase == "running" and imu_error is None and now - last_imu_at >= timeout:
                detail = (f"; last race: gate_index={race.active_gate_index}, "
                          f"boot_ms={race.sim_boot_time_ms}, start_ms={race.race_start_boot_time_ms}, "
                          f"finish_ns={race.race_finish_time_ns}" if race is not None else "; no race packet")
                imu_error = TimeoutError(f"timed out waiting for fresh IMU telemetry{detail}; "
                                         f"no native finish within {FINISH_WAIT_SECONDS:g}s")
                finish_deadline = now + FINISH_WAIT_SECONDS
                if armed:
                    armed = False
                    _disarm(vehicle)
                print("controller: IMU lost; waiting for native finish", flush=True)
            if imu_error is not None:
                # Do not resume control on recovered IMU. Keep the receive path
                # and heartbeat alive only to resolve a possibly delayed finish.
                if now >= finish_deadline:
                    raise imu_error
            elif phase == "running" and state is not None:
                if controller_deadline is None:
                    controller_deadline = now + 10
                try:
                    control = controller.update(state)
                except StopIteration:
                    return
                now = time.monotonic()
                if signals.update(controller.race, now) == "finished":
                    print("race: finished", flush=True)
                    return
                if now - state.received_at > timeout:
                    raise TimeoutError("controller returned a command for stale IMU telemetry")
                if control is None:
                    if armed:
                        raise ValueError("controller returned no command after starting")
                    if now >= controller_deadline:
                        raise TimeoutError("controller did not receive its required startup telemetry")
                else:
                    vehicle.validate(control)
                    if not armed:
                        armed = True
                        vehicle.arm()
                        print(f"controller: running at {hz:g} Hz", flush=True)
                    vehicle.send(control)
            elif phase != "running" and now >= startup_deadline:
                raise TimeoutError("race never reported GO before the startup deadline")
            if now >= next_heartbeat:
                client.heartbeat()
                next_heartbeat = now + 0.5
            next_tick += period
            if next_tick <= now:
                next_tick = now + period  # skip missed ticks; never send catch-up bursts
    finally:
        if armed:
            _disarm(vehicle)


def _disarm(vehicle):
    try:
        with suppress(OSError):
            vehicle.send(BodyRates())
    finally:
        with suppress(OSError):
            vehicle.disarm()


def _check_process(process):
    if process is not None and (status := process.poll()) is not None:
        raise RuntimeError(f"simulator exited with status {status}")


def _check_simulator_ports():
    # The client reserves the controller ports separately.
    # Refuse an existing simulator before the Wine launcher can touch its prefix.
    for port in (14560, 5601):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind(("127.0.0.1", port))
            except OSError as error:
                raise OSError(f"UDP {port} is in use; stop the existing simulator first") from error


def _stop_process(process):
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        process.terminate()  # The existing launcher stops its own Wine prefix.
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def run_session(controller: BaseController, target, hz=50.0, simulator_args=(), startup_timeout=120.0):
    """Start -> wait for telemetry -> drive -> stop, owning only this child process."""
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("hz must be positive and finite")
    if not math.isfinite(startup_timeout) or startup_timeout <= 0:
        raise ValueError("startup timeout must be positive and finite")
    launchers = {
        "vq1.r1": ["run_vq1.sh"],
        "vq2.r1": ["run_vq2.sh", "--mode", "r1"],
        "vq2.r2": ["run_vq2.sh", "--mode", "r2"],
    }
    if target not in launchers:
        raise ValueError(f"unsupported simulator: {target}")
    if target not in getattr(controller, "targets", launchers):
        raise ValueError(f"this controller does not support {target}")
    command = launchers[target].copy()
    command[0] = str(Path(__file__).resolve().parent / command[0])
    vehicle, client = controller.vehicle, controller.client
    process = None
    try:
        _check_simulator_ports()
        client.open()  # Reserve both receive ports before launching anything.
        print(f"{target}: starting", flush=True)
        process = subprocess.Popen([*command, *simulator_args], start_new_session=True)
        deadline = time.monotonic() + startup_timeout
        next_heartbeat = 0.0
        while True:
            _check_process(process)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{target} did not produce IMU telemetry within {startup_timeout:g}s")
            try:
                vehicle.read(timeout=min(0.2, remaining))
                break
            except TimeoutError:
                # Heartbeats can arrive before the pawn begins publishing IMU.
                now = time.monotonic()
                if client.connected and now >= next_heartbeat:
                    client.heartbeat()
                    next_heartbeat = now + 0.5
        _check_process(process)
        print(f"{target}: ready", flush=True)
        _drive(controller, hz, process=process, startup_deadline=deadline)
    finally:
        try:
            vehicle.disconnect()
        finally:
            _stop_process(process)
            if process is not None:
                print(f"{target}: stopped", flush=True)


def main(argv=None):
    names = sorted(p.stem for p in Path(controllers.__file__).parent.glob("*.py")
                   if not p.name.startswith("_"))
    parser = argparse.ArgumentParser(prog="control", description="Run a controller against the running VQ1 or VQ2 simulator.")
    parser.add_argument("controller", choices=names)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--simulator", choices=("vq1.r1", "vq2.r1", "vq2.r2"))
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    argv = list(sys.argv[1:] if argv is None else argv)
    boundary = argv.index("--") if "--" in argv else len(argv)
    args = parser.parse_args(argv[:boundary])
    simulator_args = argv[boundary + 1:]
    if simulator_args and args.simulator is None:
        parser.error("simulator arguments require --simulator")
    controller = importlib.import_module(f"{controllers.__name__}.{args.controller}").Controller()
    if not isinstance(controller, BaseController):
        parser.error("Controller must inherit BaseController")

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        if args.simulator:
            run_session(controller, args.simulator, args.hz, simulator_args, args.startup_timeout)
        else:
            print(f"{args.controller}: waiting for simulator; {args.hz:g} Hz. Ctrl+C stops control.", flush=True)
            run(controller, args.hz)
    except KeyboardInterrupt:
        return
    except (OSError, ValueError, TypeError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
