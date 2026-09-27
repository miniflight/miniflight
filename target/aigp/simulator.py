"""Run an AI-GP simulator with one Python controller."""

import argparse
from collections import deque
from contextlib import ExitStack, contextmanager, nullcontext, suppress
from dataclasses import dataclass, replace
import fcntl
import hashlib
import importlib
import math
import os
from pathlib import Path
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tarfile
import tempfile
import time
from types import MappingProxyType
import zipfile

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from miniflight import (Attitude, BodyRates, Command, Frame, Motion, MotorOutputs,
                       Ned, PositionNed, State, Vehicle, VelocityNed)
from target import Target
from target.aigp.controllers import BaseController


BASE = Path(__file__).resolve().parent
BINARIES = Path("FlightSim/Binaries/Win64")
SHIPPING = BINARIES / "DCGame-Win64-Shipping.exe"
PAK = Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak")
REQUIRED = (SHIPPING, BINARIES / "dwmapi.dll", BINARIES / "UE4SS.dll", PAK)
VERSIONS = {
    "vq1": {
        "root": Path("AI-GP Simulator v1.0.3391-VQ1/AIGP_VQ1_3391"),
        "legacy": "3a6923f2207a45bf64345b096d2bbd2a789916e32d1fb55beb15417b23003122",
        "hashes": {},
    },
    "vq2": {
        "root": Path("AI-GP Simulator v1.0.3391-VQ2/AIGP_VQ2_3391"),
        "legacy": "3d6527764f43862ad7860694f0783c6f4332eb87b8ccad7bd4c2bb376ce0702e",
        "hashes": {
            SHIPPING: "68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b",
            PAK: "5d424b4ee0de36053914461da56696cfff10c1ed9fab2c6bd883ace58e85883f",
        },
    },
}

FINISH_WAIT_SECONDS = 5.0
TARGETS = {
    "vq1.r1": ("vq1", "r1", "MAP_anduril_master"),
    "vq2.r1": ("vq2", "r1", "MAP_anduril_master"),
    "vq2.r2": ("vq2", "r2", "MAP_arsenal_master"),
}


@dataclass(frozen=True)
class RaceStatus:
    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int  # unchanged wire value
    received_at: float = 0.0  # host monotonic seconds

    @property
    def started(self):
        return self.race_start_boot_time_ms >= 0 and self.sim_boot_time_ms >= self.race_start_boot_time_ms

    @property
    def finished(self):
        return self.race_finish_time_ns >= 0


class AIGPSimulator:
    """Run one controller; own its connection, clock, and race lifecycle."""

    def __init__(self, controller: BaseController, target="vq1.r1", hz=50.0,
                 timeout=1.0, startup_timeout=120.0, client=None):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("hz must be positive and finite")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        if not math.isfinite(startup_timeout) or startup_timeout <= 0:
            raise ValueError("startup timeout must be positive and finite")
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
        self.vehicle = Vehicle(self.client)
        self.status = None
        self.phase = "waiting"
        self.armed = False

    def rollout(self, attach=False, simulator_args=()):
        """Own the connection, optional simulator process, and controller loop."""
        try:
            if attach:
                self.vehicle.connect()
            else:
                self.client.open()
            session = nullcontext(None) if attach else launch(self.target, simulator_args)
            with session as self._process:
                now = time.monotonic()
                self._next_tick = self._next_heartbeat = self._last_imu_at = now
                self._startup_deadline = now + self.startup_timeout
                self._controller_deadline = None
                self.status, self.phase = None, "waiting"
                try:
                    while self.step():
                        self._tick()
                    return self.status if self.phase == "finished" else None
                finally:
                    self.stop()
        finally:
            self.vehicle.disconnect()

    def step(self):
        """Receive and control once; return whether the active rollout should continue."""
        state = self._receive()
        now = time.monotonic()
        if self.phase == "finished":
            return False
        if self.phase != "running":
            if now >= self._startup_deadline:
                raise TimeoutError("race never reported GO before the startup deadline")
            return True
        if now - self._last_imu_at >= self.timeout:
            self._wait_for_finish()
            return False
        if state is None:
            return True

        if self._controller_deadline is None:
            self._controller_deadline = now + 10
        try:
            command = self.control_step(state, self.status.active_gate_index)
        except StopIteration:
            return False
        now = time.monotonic()
        self._update_status(self.client.race_status, now)
        if self.phase == "finished":
            return False
        if now - state.received_at > self.timeout:
            raise TimeoutError("controller returned a command for stale IMU telemetry")
        if command is None:
            if self.armed:
                raise ValueError("controller returned no command after starting")
            if now >= self._controller_deadline:
                raise TimeoutError("controller did not receive its required startup telemetry")
            return True
        self.vehicle.validate(command)
        if not self.armed:
            self.armed = True
            self.vehicle.arm()
            print(f"controller: running at {self.hz:g} Hz", flush=True)
        self.vehicle.send(command)
        return True

    def control_step(self, state: State, gate_index: int):
        now = time.monotonic()

        def fresh(sample):
            return sample if sample is not None and now - sample.received_at <= self.timeout else None

        observations = replace(state, motion=fresh(state.motion), attitude=fresh(state.attitude),
                               motors=fresh(state.motors), frame=fresh(state.frame))
        return self.controller.update(observations, gate_index)

    def stop(self):
        if not self.armed:
            return
        self.armed = False
        try:
            with suppress(OSError):
                self.vehicle.send(BodyRates())
        finally:
            with suppress(OSError):
                self.vehicle.disarm()

    def _receive(self):
        try:
            state = self.vehicle.read(timeout=min(self.timeout, 0.1))
            self._last_imu_at = state.received_at
        except TimeoutError:
            state = None
        self._update_status(self.client.race_status, time.monotonic())
        # The receive above may contain the final packet from an exited simulator.
        if self.phase != "finished":
            _check_process(self._process)
        return state

    def _update_status(self, status, now):
        if self.phase == "finished" or status is None:
            return
        previous, phase = self.status, self.phase
        if phase == "running" and (status.race_start_boot_time_ms != previous.race_start_boot_time_ms
                                   or status.sim_boot_time_ms < previous.sim_boot_time_ms
                                   or status.active_gate_index < previous.active_gate_index):
            raise RuntimeError("race reset during control; start a new run")
        self.status = status
        if status.finished:
            self.phase = "finished"
        elif phase != "running":
            if status.race_start_boot_time_ms < 0:
                self.phase = "waiting"
            elif not status.started:
                self.phase = "countdown"
            elif now - status.received_at <= 1.0:
                self.phase = "running"
            else:
                self.phase = "waiting"
        if self.phase != phase:
            print(f"race: {self.phase}", flush=True)
        if self.phase == "running" and (phase != "running" or status.active_gate_index != previous.active_gate_index):
            print(f"race: gate_index={status.active_gate_index}", flush=True)

    def _tick(self):
        now = time.monotonic()
        if self.client.connected and now >= self._next_heartbeat:
            self.client.heartbeat()
            self._next_heartbeat = now + 0.5
        self._next_tick += 1.0 / self.hz
        if self._next_tick <= now:
            self._next_tick = now + 1.0 / self.hz
        time.sleep(max(0.0, self._next_tick - time.monotonic()))

    def _wait_for_finish(self):
        """After IMU loss, disarm and receive only; recovery cannot resume control."""
        self.stop()
        print("controller: IMU lost; waiting for native finish", flush=True)
        status = self.status
        error = TimeoutError(f"timed out waiting for fresh IMU telemetry; last race: "
                             f"gate_index={status.active_gate_index}, boot_ms={status.sim_boot_time_ms}, "
                             f"start_ms={status.race_start_boot_time_ms}, finish_ns={status.race_finish_time_ns}; "
                             f"no native finish within {FINISH_WAIT_SECONDS:g}s")
        deadline = time.monotonic() + FINISH_WAIT_SECONDS
        while self.phase != "finished":
            self._tick()
            self._receive()
            if self.phase != "finished" and time.monotonic() >= deadline:
                raise error


class SimulatorClient(Target):
    """UDP transport for VQ1 and VQ2. Connecting never launches, arms, or resets."""

    commands = frozenset((BodyRates, PositionNed, VelocityNed))

    def __init__(self, port=14550, camera_port=5600):
        self.port = port
        self.camera_port = camera_port
        self._socket = None
        self._vision = None
        self._peer = None
        self._target = None
        self._telemetry = {}
        self._received_at = {}
        self._messages = deque(maxlen=2048)
        self.messages = ()
        self._camera = _Camera()
        self._race_status = None
        self._last_imu = None

    @property
    def connected(self):
        return self._peer is not None

    @property
    def race_status(self):
        """Latest race packet, even when read() is waiting for a new IMU sample."""
        return self._race_status

    @property
    def telemetry(self):
        """Raw MAVLink diagnostics, separate from the vehicle state."""
        return MappingProxyType(self._telemetry.copy())

    @property
    def received_at(self):
        return MappingProxyType(self._received_at.copy())

    def open(self):
        """Reserve the UDP ports without waiting for or commanding the simulator."""
        if self._socket is not None:
            raise RuntimeError("client is already open")
        self._telemetry.clear()
        self._received_at.clear()
        self._messages.clear()
        self.messages = ()
        self._camera = _Camera()
        self._race_status = self._last_imu = None
        self._boot = time.monotonic()
        self._mav = mavlink.MAVLink(self, srcSystem=255, srcComponent=191)
        self._mav.robust_parsing = True
        try:
            self._socket = self._bind(self.port)
            if self.camera_port is not None:
                self._vision = self._bind(self.camera_port)
        except BaseException:
            self.disconnect()
            raise

    def connect(self, timeout=10.0):
        if self.connected:
            raise RuntimeError("client is already connected")
        if self._socket is None:
            self.open()
        try:
            deadline = time.monotonic() + timeout
            while self._peer is None:
                self._wait(deadline, "simulator heartbeat")
        except BaseException:
            self.disconnect()
            raise

    @staticmethod
    def _bind(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # No SO_REUSEADDR: a second client must not steal the first one's packets.
            sock.bind(("127.0.0.1", port))
            sock.setblocking(False)
            return sock
        except BaseException:
            sock.close()
            raise

    def disconnect(self):
        for sock in (self._socket, self._vision):
            if sock is not None:
                sock.close()
        self._socket = self._vision = self._peer = self._target = None

    def write(self, packet):
        if self._socket is None or self._peer is None:
            raise RuntimeError("client is not connected")
        return self._socket.sendto(packet, self._peer)

    def poll(self, timeout=0.0):
        if self._socket is None:
            raise RuntimeError("client is not connected")
        sockets = [s for s in (self._socket, self._vision) if s is not None]
        # Bound each drain so continuous traffic cannot starve the controller.
        for _ in range(128):
            ready, _, _ = select.select(sockets, [], [], timeout)
            if not ready:
                break
            timeout = 0.0
            for sock in ready:
                packet, peer = sock.recvfrom(65536)
                now = time.monotonic()
                if sock is self._vision:
                    self._camera.receive(packet, now)
                    continue
                if self._peer is not None and peer != self._peer:
                    continue
                for message in self._mav.parse_buffer(packet) or ():
                    self._receive(message, peer, now)

    def _receive(self, message, peer, now):
        kind = message.get_type()
        if kind == "BAD_DATA":
            return
        source = (message.get_srcSystem(), message.get_srcComponent())
        if self._peer is None:
            if kind != "HEARTBEAT":
                return
            self._peer, self._target = peer, source
        if source[0] != self._target[0]:
            return
        if kind == "HIGHRES_IMU":
            values = (message.xacc, message.yacc, message.zacc,
                      message.xgyro, message.ygyro, message.zgyro)
            if not all(math.isfinite(v) for v in values):
                return
            previous = self._telemetry.get(kind)
            if previous is not None and message.time_usec <= previous.time_usec:
                return
        self._telemetry[kind] = message
        self._received_at[kind] = now
        self._messages.append(message)
        if kind == "ENCAPSULATED_DATA" and message.data[0] == 1:
            race = RaceStatus(*struct.unpack_from("<BQqqIq", bytes(message.data))[1:], received_at=now)
            if (self._race_status is not None and race.sim_boot_time_ms < self._race_status.sim_boot_time_ms
                    and not self._race_status.started and not race.started):
                # The native sensor clock can restart before GO.
                self._telemetry.pop("HIGHRES_IMU", None)
                self._received_at.pop("HIGHRES_IMU", None)
                self._last_imu = None
            self._race_status = race

    def _wait(self, deadline, description):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"timed out waiting for {description}")
        self.poll(min(remaining, 0.1))

    def read(self, timeout=1.0):
        """Return a snapshot with a new IMU sample, or time out. Never invent telemetry."""
        deadline = time.monotonic() + timeout
        self.poll()
        while self._telemetry.get("HIGHRES_IMU") is self._last_imu:
            self._wait(deadline, "fresh IMU telemetry")
        imu = self._telemetry["HIGHRES_IMU"]
        if time.monotonic() - self._received_at["HIGHRES_IMU"] > timeout:
            raise TimeoutError("IMU telemetry is stale")
        stamp = imu.time_usec * 1e-6
        dt = 0.0 if self._last_imu is None else stamp - self._last_imu.time_usec * 1e-6
        self._last_imu = imu
        state = State(
            stamp, dt, (imu.xacc, imu.yacc, imu.zacc), (imu.xgyro, imu.ygyro, imu.zgyro),
            self._received_at["HIGHRES_IMU"], self._camera.latest,
            self._motion(), self._attitude(), self._motors(),
        )
        self.messages = tuple(self._messages)
        self._messages.clear()
        return state

    def _motion(self):
        message = self._telemetry.get("LOCAL_POSITION_NED")
        if message is None:
            return None
        return Motion(message.time_boot_ms * 1e-3, self._received_at["LOCAL_POSITION_NED"],
                      Ned(message.x, message.y, message.z), Ned(message.vx, message.vy, message.vz))

    def _attitude(self):
        message = self._telemetry.get("ATTITUDE")
        if message is None:
            return None
        return Attitude(message.time_boot_ms * 1e-3, self._received_at["ATTITUDE"],
                        message.roll, message.pitch, message.yaw)

    def _motors(self):
        message = self._telemetry.get("ACTUATOR_OUTPUT_STATUS")
        if message is None:
            return None
        return MotorOutputs(message.time_usec * 1e-6, self._received_at["ACTUATOR_OUTPUT_STATUS"],
                            tuple(message.actuator), message.active)

    def send(self, control: Command):
        if isinstance(control, PositionNed):
            self._ned(control.north, control.east, control.down, velocity=False)
            return
        if isinstance(control, VelocityNed):
            self._ned(control.north, control.east, control.down, velocity=True)
            return
        if not isinstance(control, BodyRates):
            raise TypeError("expected BodyRates, PositionNed or VelocityNed")
        self._mav.set_attitude_target_send(
            self._time_ms(), *self._target,
            mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE | 16,  # AI-GP rad/s extension
            [1.0, 0.0, 0.0, 0.0],
            control.roll_rate, control.pitch_rate, control.yaw_rate, control.thrust,
        )

    def heartbeat(self):
        self._mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, mavlink.MAV_STATE_ACTIVE)

    def arm(self):
        self._set_armed(True)

    def disarm(self):
        self._set_armed(False)

    def _set_armed(self, armed):
        self._mav.command_long_send(*self._target, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    0, int(armed), 0, 0, 0, 0, 0, 0)

    def _time_ms(self):
        return int((time.monotonic() - self._boot) * 1000) & 0xffffffff

    def _ned(self, north, east, down, velocity):
        if not all(math.isfinite(v) for v in (north, east, down)):
            raise ValueError("NED setpoints must be finite")
        mask = (mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        if velocity:
            mask |= (mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
                     | mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
                     | mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
        else:
            mask |= (mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                     | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                     | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE)
        position = (0, 0, 0) if velocity else (north, east, down)
        speed = (north, east, down) if velocity else (0, 0, 0)
        self._mav.set_position_target_local_ned_send(
            self._time_ms(), *self._target, mavlink.MAV_FRAME_LOCAL_NED,
            mask, *position, *speed, 0, 0, 0, 0, 0,
        )


class _Camera:
    """Assemble the simulator's UDP JPEG packets; discard incomplete frames."""

    HEADER = struct.Struct("<IHHIIQ")
    MAX_BYTES = 8 * 1024 * 1024
    MAX_FRAMES = 3
    MAX_AGE = 0.5

    def __init__(self):
        self.pending = {}
        self.latest = None

    def receive(self, packet: bytes, now: float):
        self.pending = {
            key: value for key, value in self.pending.items()
            if now - value[0] < self.MAX_AGE
        }
        if len(packet) < self.HEADER.size:
            return
        frame_id, index, count, size, payload_size, timestamp = self.HEADER.unpack_from(packet)
        payload = packet[self.HEADER.size:]
        if not (0 <= index < count <= 4096 and 0 < size <= self.MAX_BYTES
                and 0 < payload_size == len(payload) <= size):
            return
        if self.latest is not None and timestamp <= self.latest.time_ns:
            return
        metadata = (count, size, timestamp)
        if frame_id not in self.pending:
            if len(self.pending) == self.MAX_FRAMES:
                del self.pending[next(iter(self.pending))]
            self.pending[frame_id] = (now, metadata, {})
        _, expected, chunks = self.pending[frame_id]
        if metadata != expected:
            del self.pending[frame_id]
            return
        chunks.setdefault(index, payload)
        if sum(map(len, chunks.values())) > size:
            del self.pending[frame_id]
            return
        if len(chunks) != count:
            return
        del self.pending[frame_id]
        jpeg = b"".join(chunks[i] for i in range(count))
        if len(jpeg) != size:
            return
        try:
            bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        except cv2.error:
            return
        if bgr is not None:
            bgr.flags.writeable = False
            self.latest = Frame(frame_id, timestamp, now, bgr)


def configuration(version):
    return {
        "main.lua": BINARIES / f"Mods/Direct{version.upper()}/Scripts/main.lua",
        "mods.txt": BINARIES / "Mods/mods.txt",
        "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
    }


def prepare(version, base=BASE):
    profile = VERSIONS[version]
    parts = [line.split() for line in (base / "archives/SHA256SUMS").read_text().splitlines()
             if line.strip() and (line.split()[-1] == f"{version}.tar.xz"
                                  or line.split()[-1].startswith(f"{version}-unlocked.tar.gz.part-"))]
    config = {Path("config") / version / name: path for name, path in configuration(version).items()}
    return install(base, version, parts, profile["root"], REQUIRED, config, profile["hashes"],
                   archive_dir=base / "archives", cache_versions=(profile["legacy"],))


def configure(base, sim, files):
    for source, relative in files.items():
        destination = sim / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(base / source, destination)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def install(base, name, parts, archive_root, required, config=None, payload_sha256=None,
            archive_dir=None, cache_versions=()):
    if not hasattr(tarfile, "data_filter"):
        raise SystemExit("Use a current Python 3.11 or newer to prepare the simulator.")
    if not parts:
        raise SystemExit(f"No archive parts listed for {name}.")
    config = config or {}
    archive_dir = base if archive_dir is None else archive_dir
    version = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / name
    marker = sim / ".installed"

    try:
        installed = (marker.is_file() and marker.read_text() in (version, *cache_versions)
                     and all((sim / path).is_file() for path in required))
    except (OSError, UnicodeError):
        installed = False
    if installed:
        configure(base, sim, config)
        return sim

    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{name}-install-", dir=runtime) as temporary:
        staging = Path(temporary)
        with ExitStack() as files:
            combined = (files.enter_context(tempfile.TemporaryFile(dir=staging))
                        if len(parts) > 1 else None)
            for expected, filename in parts:
                path = archive_dir / filename
                if not path.is_file():
                    raise SystemExit(f"Missing {filename}. Supply the archive before installing {name}.")
                digest = hashlib.sha256()
                source = files.enter_context(path.open("rb"))
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(block)
                    if combined is not None:
                        combined.write(block)
                if digest.hexdigest() != expected:
                    raise SystemExit(f"Checksum mismatch: {filename}. Download the expected archive and retry.")
                print(f"Verified {filename}", flush=True)
            if combined is None:
                combined = source
            combined.seek(0)
            if zipfile.is_zipfile(combined):
                with zipfile.ZipFile(combined) as archive:
                    archive.extractall(staging)
            else:
                combined.seek(0)
                with tarfile.open(fileobj=combined, mode="r:*") as archive:
                    archive.extractall(staging, filter="data")

        extracted = staging / archive_root
        if not all((extracted / path).is_file() for path in required):
            raise SystemExit(f"{name} archive is missing required simulator files.")
        for relative, expected in (payload_sha256 or {}).items():
            if sha256(extracted / relative) != expected:
                raise SystemExit(f"{name} archive contains the wrong {relative} build.")
        configure(base, extracted, config)
        (extracted / ".installed").write_text(version)
        backup = None
        try:
            if sim.exists() or sim.is_symlink():
                backup = Path(tempfile.mkdtemp(prefix=f"{name}-backup-", dir=runtime)) / name
                sim.rename(backup)
            extracted.rename(sim)
        except BaseException:
            if backup is not None:
                if backup.exists() or backup.is_symlink():
                    backup.rename(sim)
                backup.parent.rmdir()
            raise
        if backup is not None:
            print(f"Preserved previous {name} installation at {backup}", flush=True)
    print(f"Prepared {name} at {sim}", flush=True)
    return sim


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


def main(argv=None):
    names = sorted(p.stem for p in (BASE / "controllers").glob("*.py")
                   if not p.name.startswith("_"))
    parser = argparse.ArgumentParser(prog="simulator.py", description="Run an AI-GP simulator and a Python controller.")
    parser.add_argument("target", nargs="?", choices=(*TARGETS, "vq1"))
    parser.add_argument("--controller", choices=names)
    parser.add_argument("--prepare", choices=VERSIONS, help="extract and configure a simulator without launching it")
    parser.add_argument("--attach", action="store_true", help="control an already running simulator")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--startup-timeout", type=float)
    argv = list(sys.argv[1:] if argv is None else argv)
    boundary = argv.index("--") if "--" in argv else len(argv)
    args, simulator_args = parser.parse_known_args(argv[:boundary])
    simulator_args.extend(argv[boundary + 1:])
    if args.prepare:
        if args.target or args.controller or args.attach or simulator_args or args.hz is not None or args.startup_timeout is not None:
            parser.error("--prepare cannot be combined with run options")
        prepare(args.prepare)
        return 0
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
