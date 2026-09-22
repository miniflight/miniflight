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

from miniflight import Control, PositionNed
from target.aigp import SimulatorClient, controllers


class _RaceSignals:
    """Interpret native race timestamps; never infer GO from local elapsed time."""

    def __init__(self):
        self.start = None
        self.previous = None

    def update(self, race, now):
        if race is None:
            if self.start is not None:
                raise TimeoutError("race telemetry disappeared")
            return "waiting"
        if now - race.received_at > 1.0:
            raise TimeoutError("race telemetry is stale")
        if self.start is not None:
            if (race.race_start_boot_time_ms != self.start
                    or race.sim_boot_time_ms < self.previous.sim_boot_time_ms
                    or race.active_gate_index < self.previous.active_gate_index):
                raise RuntimeError("race reset during control; start a new run")
        self.previous = race
        if race.race_finish_time_ns >= 0:
            return "finished"
        if race.race_start_boot_time_ms < 0:
            return "waiting"
        if race.sim_boot_time_ms < race.race_start_boot_time_ms:
            return "countdown"
        self.start = race.race_start_boot_time_ms
        return "running"


def run(controller, sim, hz=50.0, timeout=1.0):
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("hz must be positive and finite")
    try:
        sim.connect()
        _drive(controller, sim, hz, timeout)
    finally:
        sim.disconnect()


def _drive(controller, sim, hz, timeout=1.0, process=None, startup_deadline=None):
    armed = False
    period = 1.0 / hz
    next_tick = next_heartbeat = time.monotonic()
    last_imu_at = next_tick
    if startup_deadline is None:
        startup_deadline = next_tick + 120
    signals, phase, controller_deadline = _RaceSignals(), None, None
    try:
        while True:
            time.sleep(max(0.0, next_tick - time.monotonic()))
            _check_process(process)
            try:
                # Race events and heartbeats must keep moving when IMU stops.
                state = sim.read(timeout=min(timeout, 0.1))
                last_imu_at = state.received_at["HIGHRES_IMU"]
            except TimeoutError:
                state = None
            now = time.monotonic()
            race = state.race if state is not None else sim.race
            current_phase = signals.update(race, now)
            if current_phase != phase:
                print(f"race: {current_phase}", flush=True)
                phase = current_phase
            if phase == "finished":
                return
            if now - last_imu_at >= timeout:
                detail = (f"; last race: gate_index={race.active_gate_index}, "
                          f"boot_ms={race.sim_boot_time_ms}, start_ms={race.race_start_boot_time_ms}, "
                          f"finish_ns={race.race_finish_time_ns}" if race is not None else "; no race packet")
                raise TimeoutError(f"timed out waiting for fresh IMU telemetry{detail}")
            if phase == "running" and state is not None:
                if controller_deadline is None:
                    controller_deadline = now + 10
                try:
                    control = controller.update(state)
                except StopIteration:
                    return
                now = time.monotonic()
                if now - state.received_at["HIGHRES_IMU"] > timeout:
                    raise TimeoutError("controller returned a command for stale IMU telemetry")
                # A slow controller must not outlive the race sample that allowed it.
                signals.update(state.race, now)
                if control is None:
                    if armed:
                        raise ValueError("controller returned no command after starting")
                    if now >= controller_deadline:
                        raise TimeoutError("controller did not receive its required startup telemetry")
                else:
                    if not isinstance(control, (Control, PositionNed)):
                        raise TypeError("controller.update(state) must return Control or PositionNed")
                    if not armed:
                        armed = True
                        sim.arm()
                        print(f"controller: running at {hz:g} Hz", flush=True)
                    sim.send(control)
            elif phase != "running" and now >= startup_deadline:
                raise TimeoutError("race never reported GO before the startup deadline")
            if now >= next_heartbeat:
                sim.heartbeat()
                next_heartbeat = now + 0.5
            next_tick += period
            if next_tick <= now:
                next_tick = now + period  # skip missed ticks; never send catch-up bursts
    finally:
        if armed:
            try:
                with suppress(OSError):
                    sim.send(Control())
            finally:
                with suppress(OSError):
                    sim.disarm()


def _check_process(process):
    if process is not None and (status := process.poll()) is not None:
        raise RuntimeError(f"simulator exited with status {status}")


def _check_simulator_ports():
    # The controller ports are reserved separately by SimulatorClient.open().
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


def run_session(controller, target, hz=50.0, simulator_args=(), startup_timeout=120.0):
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
    sim, process = SimulatorClient(), None
    try:
        _check_simulator_ports()
        sim.open()  # Reserve both receive ports before launching anything.
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
                sim.read(timeout=min(0.2, remaining))
                break
            except TimeoutError:
                # Heartbeats can arrive before the pawn begins publishing IMU.
                now = time.monotonic()
                if sim.connected and now >= next_heartbeat:
                    sim.heartbeat()
                    next_heartbeat = now + 0.5
        _check_process(process)
        print(f"{target}: ready", flush=True)
        _drive(controller, sim, hz, process=process, startup_deadline=deadline)
    finally:
        try:
            sim.disconnect()
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

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        if args.simulator:
            run_session(controller, args.simulator, args.hz, simulator_args, args.startup_timeout)
        else:
            print(f"{args.controller}: waiting for simulator; {args.hz:g} Hz. Ctrl+C stops control.", flush=True)
            run(controller, SimulatorClient(), args.hz)
    except KeyboardInterrupt:
        return
    except (OSError, ValueError, TypeError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
