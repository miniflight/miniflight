"""Run an AI-GP simulator with one Python controller."""

import argparse
from contextlib import ExitStack, contextmanager, suppress
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

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from common.math import Quaternion, Vector3D
from miniflight import (Attitude, BodyRates, Command, Frame, Motion, MotorOutputs,
                       Ned, PositionNed, State, VelocityNed)
from target import Target
from target.aigp.controllers import BaseController, Gate


BASE = Path(__file__).resolve().parent
BINARIES = Path("FlightSim/Binaries/Win64")
SHIPPING = BINARIES / "DCGame-Win64-Shipping.exe"
PAK = Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak")
REQUIRED = (SHIPPING, BINARIES / "dwmapi.dll", BINARIES / "UE4SS.dll", PAK)
VERSIONS = {
    "vq1": (Path("AI-GP Simulator v1.0.3391-VQ1/AIGP_VQ1_3391"),
            "3a6923f2207a45bf64345b096d2bbd2a789916e32d1fb55beb15417b23003122"),
    "vq2": (Path("AI-GP Simulator v1.0.3391-VQ2/AIGP_VQ2_3391"),
            "3d6527764f43862ad7860694f0783c6f4332eb87b8ccad7bd4c2bb376ce0702e"),
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
        with ExitStack() as session:
            session.callback(self.client.disconnect)
            self.client.open()
            process = None if attach else session.enter_context(launch(self.target, simulator_args))
            session.callback(self.stop)
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
                    self.status = status
                    if status.finished:
                        print("race: finished", flush=True)
                        return status
                    previous_phase = phase
                    if phase == "startup" and status.started and now - status.received_at <= 1.0:
                        phase = "control"
                        print("race: running", flush=True)
                    if phase != "startup" and (previous_phase == "startup" or status.active_gate_index != previous.active_gate_index):
                        print(f"race: gate_index={status.active_gate_index}", flush=True)
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

                    def fresh(sample):
                        return sample if sample is not None and now - sample.received_at <= self.timeout else None

                    state = replace(state, motion=fresh(state.motion), attitude=fresh(state.attitude),
                                    motors=fresh(state.motors), frame=fresh(state.frame))
                    try:
                        command = self.controller.update(state, index, gates)
                    except StopIteration:
                        return None
                    now = time.monotonic()
                    if now - state.received_at > self.timeout:
                        raise TimeoutError("controller returned a command for stale IMU telemetry")
                    if command is None:
                        if self.armed:
                            raise ValueError("controller returned no command after starting")
                        if now >= deadline:
                            raise TimeoutError("controller did not receive its required startup telemetry")
                    else:
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

    def stop(self):
        if not self.armed:
            return
        self.armed = False
        try:
            with suppress(OSError):
                self.client.send(BodyRates())
        finally:
            with suppress(OSError):
                self.client.disarm()


class SimulatorClient(Target):
    """UDP transport for VQ1 and VQ2. Connecting never launches, arms, or resets."""

    commands = frozenset((BodyRates, PositionNed, VelocityNed))

    def __init__(self, port=14550, camera_port=5600):
        self.port = port
        self.camera_port = camera_port
        self._socket = self._vision = self._peer = self._target = None
        self._telemetry = {}
        self.telemetry = MappingProxyType(self._telemetry)  # Message name → latest raw MAVLink packet.
        self._track = _Track()
        self.race_status: RaceStatus | None = None  # Latest packet, independent of the IMU.

    @property
    def connected(self):
        return self._peer is not None

    @property
    def gates(self):
        """Cached tuple of all gates, or None. Replaced only by a complete track transfer."""
        return self._track.gates

    def open(self):
        """Reserve the UDP ports without waiting for or commanding the simulator."""
        if self._socket is not None:
            raise RuntimeError("client is already open")
        self._telemetry.clear()
        self._imu_received_at = 0.0
        self._motion = self._attitude = self._motors = None
        self._camera = _Camera()
        self._track = _Track()
        self.race_status = self._last_imu = None
        self._boot = time.monotonic()
        self._mav = mavlink.MAVLink(self, srcSystem=255, srcComponent=191)
        self._mav.robust_parsing = True
        try:
            # Exclusive binds: a second client must not steal these packets.
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("127.0.0.1", self.port))
            self._socket.setblocking(False)
            if self.camera_port is not None:
                self._vision = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._vision.bind(("127.0.0.1", self.camera_port))
                self._vision.setblocking(False)
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
            self._imu_received_at = now
        self._telemetry[kind] = message
        if kind == "LOCAL_POSITION_NED":
            values = (message.x, message.y, message.z, message.vx, message.vy, message.vz)
            self._motion = (Motion(message.time_boot_ms * 1e-3, now, Ned(*values[:3]), Ned(*values[3:]))
                            if all(math.isfinite(value) for value in values) else None)
        elif kind == "ATTITUDE":
            # Build 3391 reports pitch/yaw with the opposite signs to local NED.
            self._attitude = (Attitude(message.time_boot_ms * 1e-3, now,
                                      message.roll, -message.pitch, -message.yaw)
                              if all(math.isfinite(value) for value in (message.roll, message.pitch, message.yaw)) else None)
        elif kind == "ACTUATOR_OUTPUT_STATUS":
            self._motors = MotorOutputs(message.time_usec * 1e-6, now, tuple(message.actuator), message.active)
        elif kind == "DATA_TRANSMISSION_HANDSHAKE":
            self._track.start(message)
        elif kind == "ENCAPSULATED_DATA" and message.data[0] == 2:
            self._track.receive(message)
        if kind == "ENCAPSULATED_DATA" and message.data[0] == 1:
            race = RaceStatus(*struct.unpack_from("<BQqqIq", bytes(message.data))[1:], received_at=now)
            if (self.race_status is not None and race.sim_boot_time_ms < self.race_status.sim_boot_time_ms
                    and not self.race_status.started and not race.started):
                # The native sensor clock can restart before GO.
                self._telemetry.pop("HIGHRES_IMU", None)
                self._last_imu = None
            self.race_status = race

    def _wait(self, deadline, description):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"timed out waiting for {description}")
        self.poll(min(remaining, 0.1))

    def read(self, timeout=1.0):
        """Read one fresh IMU sample plus the latest optional motion, attitude, motors and image.

        time and dt are IMU seconds; acceleration is body m/s² and gyro is body rad/s.
        Each optional sample keeps its own timestamp. Gate geometry is a separate cache.
        """
        deadline = time.monotonic() + timeout
        self.poll()
        while self._telemetry.get("HIGHRES_IMU") is self._last_imu:
            self._wait(deadline, "fresh IMU telemetry")
        imu = self._telemetry["HIGHRES_IMU"]
        if time.monotonic() - self._imu_received_at > timeout:
            raise TimeoutError("IMU telemetry is stale")
        stamp = imu.time_usec * 1e-6
        dt = 0.0 if self._last_imu is None else stamp - self._last_imu.time_usec * 1e-6
        self._last_imu = imu
        # Angular rates on the AIGP wire have the opposite signs to body FRD.
        return State(
            time=stamp,
            dt=dt,
            acceleration=(imu.xacc, imu.yacc, imu.zacc),
            gyro=(-imu.xgyro, -imu.ygyro, -imu.zgyro),
            received_at=self._imu_received_at,
            frame=self._camera.latest,
            motion=self._motion,
            attitude=self._attitude,
            motors=self._motors,
        )

    def send(self, command: Command):
        """Write one NED position, NED velocity, or body-rate-and-thrust command."""
        if not isinstance(command, Command):
            raise TypeError("expected BodyRates, PositionNed or VelocityNed")
        time_ms = int((time.monotonic() - self._boot) * 1000) & 0xffffffff
        if isinstance(command, BodyRates):
            self._mav.set_attitude_target_send(
                time_ms, *self._target,
                mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE | 16,  # AI-GP rad/s extension
                [1.0, 0.0, 0.0, 0.0],
                # The simulator's angular-rate wire axes are opposite to FRD.
                -command.roll_rate, -command.pitch_rate, -command.yaw_rate, command.thrust,
            )
        else:
            vector = (command.north, command.east, command.down)
            position = vector if isinstance(command, PositionNed) else (0, 0, 0)
            velocity = vector if isinstance(command, VelocityNed) else (0, 0, 0)
            # 1 ignores a field: xyz bits 0..2, velocity 3..5, acceleration 6..8, yaw 10..11.
            mask = 0b110111111000 if isinstance(command, PositionNed) else 0b110111000111
            self._mav.set_position_target_local_ned_send(
                time_ms, *self._target, mavlink.MAV_FRAME_LOCAL_NED,
                mask, *position, *velocity, 0, 0, 0, 0, 0,
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


class _Track:
    """Assemble the advertised track before publishing immutable gate geometry."""

    GATE = struct.Struct("<H9f")
    CHUNK_BYTES = 250  # ENCAPSULATED_DATA minus its type and transfer ID.
    MAX_GATES = 1024

    def __init__(self):
        self.transfer = None
        self.chunks = {}
        self.gates = None

    def start(self, message):
        size, packets = message.size, message.packets
        if (not 2 <= size <= 2 + self.MAX_GATES * self.GATE.size
                or packets != (size + self.CHUNK_BYTES - 1) // self.CHUNK_BYTES):
            return
        transfer = (message.width, size, packets)  # width carries the vendor's transfer ID.
        if transfer != self.transfer:
            self.transfer, self.chunks = transfer, {}

    def receive(self, message):
        if self.transfer is None:
            return
        payload = bytes(message.data)
        transfer_id, size, packets = self.transfer
        if struct.unpack_from("<H", payload, 1)[0] != transfer_id:
            return
        index = message.seqnr
        if not 0 <= index < packets:
            return
        length = min(self.CHUNK_BYTES, size - index * self.CHUNK_BYTES)
        self.chunks[index] = payload[3:3 + length]
        if len(self.chunks) != packets:
            return
        data = b"".join(self.chunks[i] for i in range(packets))
        self.transfer, self.chunks = None, {}
        count, = struct.unpack_from("<H", data)
        if not 0 < count <= self.MAX_GATES or len(data) != 2 + count * self.GATE.size:
            return
        gates = []
        for index, row in enumerate(self.GATE.iter_unpack(data[2:])):
            gate_id, north, east, down, w, x, y, z, width, height = row
            if gate_id != index or not all(math.isfinite(value) for value in row[1:]) or width <= 0 or height <= 0:
                return
            norm = math.hypot(w, x, y, z)
            if not 0.99 <= norm <= 1.01:
                return
            orientation = tuple(value / norm for value in (w, x, y, z))
            # The published origin is at the gate base; offset to the opening center.
            origin = Ned(north, east, down)
            offset = Quaternion(*orientation).rotate(Vector3D(0, 0, -height / 2)).v
            center = Ned(*(float(p + d) for p, d in zip(origin, offset)))
            gates.append(Gate(gate_id, center, orientation, width, height, origin=origin))
        self.gates = tuple(gates)


class _Camera:
    """Assemble the simulator's UDP JPEG packets; discard incomplete frames."""

    HEADER = struct.Struct("<IHHIIQ")
    MAX_BYTES = 8 * 1024 * 1024
    MAX_AGE = 0.5

    def __init__(self):
        self.frame = None
        self.started_at = 0.0
        self.chunks = {}
        self.latest = None

    def receive(self, packet: bytes, now: float):
        if len(packet) < self.HEADER.size:
            return
        frame_id, index, count, size, payload_size, timestamp = self.HEADER.unpack_from(packet)
        payload = packet[self.HEADER.size:]
        if not (0 <= index < count <= 4096 and 0 < size <= self.MAX_BYTES
                and 0 < payload_size == len(payload) <= size):
            return
        if self.latest is not None and timestamp <= self.latest.time_ns:
            return
        frame = (frame_id, count, size, timestamp)
        if self.frame is None or timestamp > self.frame[3] or now - self.started_at > self.MAX_AGE:
            self.frame, self.started_at, self.chunks = frame, now, {}
        if frame != self.frame:
            return
        self.chunks[index] = payload
        if sum(map(len, self.chunks.values())) > size:
            self.frame, self.chunks = None, {}
            return
        if len(self.chunks) != count:
            return
        jpeg = b"".join(self.chunks[i] for i in range(count))
        self.frame, self.chunks = None, {}
        if len(jpeg) != size:
            return
        try:
            bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        except cv2.error:
            return
        if bgr is not None:
            bgr.flags.writeable = False
            self.latest = Frame(frame_id, timestamp, now, bgr)


def prepare(version, base=BASE):
    """Verify and extract the selected tar archive once, then copy its three config files."""
    root, legacy_stamp = VERSIONS[version]
    parts = [line.split() for line in (base / "archives/SHA256SUMS").read_text().splitlines()
             if line.strip() and line.split()[-1] == f"{version}.tar.xz"]
    expected, filename = parts[0]
    stamp = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / version
    marker = sim / ".installed"
    installed = (marker.is_file() and marker.read_text() in (stamp, legacy_stamp)
                 and all((sim / path).is_file() for path in REQUIRED))
    if not installed:
        if sim.exists():
            raise FileExistsError(f"Incomplete {sim}; move it aside before preparing again")
        archive = base / "archives" / filename
        if sha256(archive) != expected:
            raise ValueError(f"Checksum mismatch: {filename}")
        runtime.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=f"{version}-install-", dir=runtime) as temporary:
            with tarfile.open(archive, "r:xz") as source:
                source.extractall(temporary, filter="data")
            extracted = Path(temporary) / root
            if not all((extracted / path).is_file() for path in REQUIRED):
                raise ValueError(f"{filename} is missing required simulator files")
            (extracted / ".installed").write_text(stamp)
            extracted.rename(sim)
    for name, relative in {
        "main.lua": BINARIES / f"Mods/Direct{version.upper()}/Scripts/main.lua",
        "mods.txt": BINARIES / "Mods/mods.txt",
        "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
    }.items():
        destination = sim / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(base / "config" / version / name, destination)
    return sim


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
    # Refuse an existing simulator before touching its Wine prefix.
    for port in (14560, 5601):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind(("127.0.0.1", port))
            except OSError as error:
                raise OSError(f"UDP {port} is in use; stop the existing simulator first") from error
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
    parser = argparse.ArgumentParser(prog="simulator.py", allow_abbrev=False,
                                     description="Run an AI-GP simulator and a Python controller.")
    parser.add_argument("target", nargs="?", choices=(*TARGETS, "vq1"))
    parser.add_argument("--controller", choices=names)
    parser.add_argument("--prepare", choices=VERSIONS, help="extract and configure a simulator without launching it")
    parser.add_argument("--attach", action="store_true", help="control an already running simulator")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--startup-timeout", type=float)
    argv = list(sys.argv[1:] if argv is None else argv)
    boundary = argv.index("--") if "--" in argv else len(argv)
    args = parser.parse_args(argv[:boundary])
    simulator_args = argv[boundary + 1:]
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
