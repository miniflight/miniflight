import argparse
from contextlib import contextmanager, suppress
import fcntl
import importlib
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import time

from miniflight import BodyRates
from target.aigp import controllers
from target.aigp.controllers import BaseController
from target.aigp.install import BASE, SHIPPING, prepare


FINISH_WAIT_SECONDS = 5.0
TARGETS = {
    "vq1.r1": ("vq1", "r1", "MAP_anduril_master"),
    "vq2.r1": ("vq2", "r1", "MAP_anduril_master"),
    "vq2.r2": ("vq2", "r2", "MAP_arsenal_master"),
}


def wine_commands():
    if sys.platform not in ("darwin", "linux"):
        raise RuntimeError("the runner requires macOS or Linux")
    configured = os.environ.get("WINE")
    if configured:
        wine = shutil.which(configured)
    elif sys.platform == "darwin":
        wine = shutil.which("/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wine64")
    else:
        wine = shutil.which("wine64") or shutil.which("wine")
    if wine is None:
        raise FileNotFoundError("Wine not found; install it or set WINE to its executable")
    server = os.environ.get("WINESERVER", str(Path(wine).with_name("wineserver")))
    server = shutil.which(server)
    if server is None and not configured and "WINESERVER" not in os.environ and sys.platform == "linux":
        server = shutil.which("wineserver")
    if server is None:
        raise FileNotFoundError("wineserver not found; set WINESERVER to the matching executable")
    return wine, server


@contextmanager
def launch(target, simulator_args=()):
    """Own one Wine prefix and process group, from preparation through shutdown."""
    if target not in TARGETS:
        raise ValueError(f"unsupported simulator: {target}")
    _check_simulator_ports()
    wine, server = wine_commands()
    version, mode, level = TARGETS[target]
    prefix = BASE / ".runtime" / f"{version}-wine"
    prefix.mkdir(parents=True, exist_ok=True)
    with (prefix / ".runner.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f"{version} is already running") from None
        sim = prepare(version, BASE)
        env = dict(os.environ, WINEPREFIX=str(prefix), WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer=")
        if version == "vq2":
            env["MINIFLIGHT_VQ2_MODE"] = mode
        if sys.platform == "darwin":
            env["WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER"] = "1"
        command = [wine, str(sim / SHIPPING), f"/Game/levelsMaster/{level}?game=/Script/DCGame.GameModeRaceBase",
                   "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-NoSplash", *simulator_args]
        process = None
        try:
            stopped = subprocess.run([server, "-k"], env=env, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL, timeout=10)
            if stopped.returncode not in (0, 1):  # Wine returns 1 when no server is running.
                stopped.check_returncode()
            print(f"{target}: starting", flush=True)
            process = subprocess.Popen(command, cwd=sim, env=env, start_new_session=True)
            yield process
        finally:
            # Also clean up when Wine starts children but its launcher exits.
            with suppress(OSError, subprocess.TimeoutExpired):
                subprocess.run([server, "-k"], env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=10)
            _stop_process(process)
            if process is not None:
                print(f"{target}: stopped", flush=True)


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
        process.terminate()
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
    if target not in TARGETS:
        raise ValueError(f"unsupported simulator: {target}")
    if target not in getattr(controller, "targets", TARGETS):
        raise ValueError(f"this controller does not support {target}")
    vehicle, client = controller.vehicle, controller.client
    try:
        client.open()  # Reserve both receive ports before launching anything.
        with launch(target, simulator_args) as process:
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
        vehicle.disconnect()


def main(argv=None):
    names = sorted(p.stem for p in Path(controllers.__file__).parent.glob("*.py")
                   if not p.name.startswith("_"))
    parser = argparse.ArgumentParser(prog="run", description="Run an AI-GP simulator and a Python controller.")
    parser.add_argument("target", nargs="?", choices=(*TARGETS, "vq1"))
    parser.add_argument("--controller", choices=names)
    parser.add_argument("--attach", action="store_true", help="control an already running simulator")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--startup-timeout", type=float)
    argv = list(sys.argv[1:] if argv is None else argv)
    boundary = argv.index("--") if "--" in argv else len(argv)
    args, simulator_args = parser.parse_known_args(argv[:boundary])
    simulator_args.extend(argv[boundary + 1:])
    if args.target == "vq1":
        args.target = "vq1.r1"
    if args.attach:
        if args.target or simulator_args or args.startup_timeout is not None or not args.controller:
            parser.error("--attach requires --controller and no simulator arguments")
    elif args.target is None:
        parser.error("choose vq1.r1 vq2.r1 or vq2.r2")
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
            controller = importlib.import_module(f"{controllers.__name__}.{args.controller}").Controller()
            if not isinstance(controller, BaseController):
                parser.error("Controller must inherit BaseController")
            if args.attach:
                run(controller, hz)
            else:
                run_session(controller, args.target, hz, simulator_args, startup_timeout)
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
