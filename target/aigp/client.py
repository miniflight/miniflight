"""Receive asynchronous AI-GP packets; read observations and encode commands."""

from dataclasses import dataclass
import math
import select
import socket
import struct
import time
from types import MappingProxyType

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from common.math import Quaternion, Vector3D
from miniflight import (Attitude, BodyRates, Command, Frame, Motion, MotorOutputs,
                       Ned, PositionNed, State, VelocityNed)
from target import Target
from target.aigp.controllers import Gate


@dataclass(frozen=True)
class RaceStatus:
    """Latest native race packet, received independently of sensor packets."""

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


class SimulatorClient(Target):
    """UDP transport for VQ1 and VQ2. Connecting never launches, arms, or resets.

    mav and target_ids expose the wire; send converts the three flight commands.
    poll receives packets; properties only inspect caches. read waits for a new
    IMU sample and joins the latest optional samples, each with its own clock.
    telemetry holds the latest raw packet per message name, not packet history.
    """

    commands = frozenset((BodyRates, PositionNed, VelocityNed))

    def __init__(self, port=14550, camera_port=5600):
        self.port = port
        self.camera_port = camera_port
        self._socket = self._vision = self._peer = self.mav = self.target_ids = None
        self._telemetry = {}
        self.telemetry = MappingProxyType(self._telemetry)  # Message name → latest raw MAVLink packet.
        self._track = _Track()
        self.race_status: RaceStatus | None = None  # Latest packet, independent of the IMU.

    @property
    def connected(self):
        return self._peer is not None

    @property
    def gates(self):
        """Cached course geometry; replaced only by a complete usable transfer."""
        return self._track.gates

    @property
    def gates_received_at(self):
        """Host monotonic receipt of that complete transfer, or None. Does not poll."""
        return self._track.received_at

    def open(self):
        """Reserve the UDP ports without waiting for or commanding the simulator."""
        if self._socket is not None:
            raise RuntimeError("client is already open")
        self._telemetry.clear()
        self._camera = _Camera()
        self._track = _Track()
        self.race_status = self._last_imu = None
        self._boot = time.monotonic()
        self.mav = mavlink.MAVLink(self, srcSystem=255, srcComponent=191)
        self.mav.robust_parsing = True
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
        self._socket = self._vision = self._peer = self.mav = self.target_ids = None

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
                for message in self.mav.parse_buffer(packet) or ():
                    self._receive(message, peer, now)

    def _receive(self, message, peer, now):
        kind = message.get_type()
        if kind == "BAD_DATA":
            return
        source = (message.get_srcSystem(), message.get_srcComponent())
        if self._peer is None:
            if kind != "HEARTBEAT":
                return
            self._peer, self.target_ids = peer, source
        if source[0] != self.target_ids[0]:
            return
        if kind == "HIGHRES_IMU":
            values = (message.xacc, message.yacc, message.zacc,
                      message.xgyro, message.ygyro, message.zgyro)
            if not all(math.isfinite(v) for v in values):
                return
            previous = self._telemetry.get(kind)
            if previous is not None and message.time_usec <= previous.time_usec:
                return
        message._host_received_at = now  # Host metadata; MAVLink fields and packet bytes are unchanged.
        self._telemetry[kind] = message
        if kind == "DATA_TRANSMISSION_HANDSHAKE":
            # Announcement only: transfer ID, byte count, fragment count.
            self._track.start(message)
        elif kind == "ENCAPSULATED_DATA" and message.data[0] == 2:
            # Fragment arrival may complete a track; an IMU read never does.
            self._track.receive(message, now)
        elif kind == "ENCAPSULATED_DATA" and message.data[0] == 1:
            # Native boot/start/finish/gate values, not an IMU or host tick.
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
        """Read the newest unread IMU plus cached motion, attitude, motors and image.

        time and dt are IMU seconds; acceleration is body m/s² and gyro is body rad/s.
        Intermediate IMU samples may be skipped. Optional packets arrive separately
        and keep their own timestamps; this is not a synchronized physics step.
        Race status and gate geometry stay in their separate caches.
        """
        deadline = time.monotonic() + timeout
        self.poll()
        while self._telemetry.get("HIGHRES_IMU") is self._last_imu:
            self._wait(deadline, "fresh IMU telemetry")
        imu = self._telemetry["HIGHRES_IMU"]
        if time.monotonic() - imu._host_received_at > timeout:
            raise TimeoutError("IMU telemetry is stale")
        stamp = imu.time_usec * 1e-6
        dt = 0.0 if self._last_imu is None else stamp - self._last_imu.time_usec * 1e-6
        self._last_imu = imu
        motion = self._telemetry.get("LOCAL_POSITION_NED")
        if motion is not None:
            values = (motion.x, motion.y, motion.z, motion.vx, motion.vy, motion.vz)
            motion = (Motion(motion.time_boot_ms * 1e-3, motion._host_received_at, Ned(*values[:3]), Ned(*values[3:]))
                      if all(math.isfinite(value) for value in values) else None)
        attitude = self._telemetry.get("ATTITUDE")
        if attitude is not None:
            # Build 3391 reports pitch/yaw with the opposite signs to local NED.
            attitude = (Attitude(attitude.time_boot_ms * 1e-3, attitude._host_received_at,
                                 attitude.roll, -attitude.pitch, -attitude.yaw)
                        if all(math.isfinite(value) for value in (attitude.roll, attitude.pitch, attitude.yaw)) else None)
        motors = self._telemetry.get("ACTUATOR_OUTPUT_STATUS")
        if motors is not None:
            motors = MotorOutputs(motors.time_usec * 1e-6, motors._host_received_at, tuple(motors.actuator), motors.active)
        # Angular rates on the AIGP wire have the opposite signs to body FRD.
        return State(time=stamp, dt=dt, received_at=imu._host_received_at,
                     acceleration=(imu.xacc, imu.yacc, imu.zacc), gyro=(-imu.xgyro, -imu.ygyro, -imu.zgyro),
                     motion=motion, attitude=attitude, motors=motors, frame=self._camera.latest)

    def send(self, command: Command):
        """Write one NED position, NED velocity, or body-rate-and-thrust command."""
        if not isinstance(command, Command):
            raise TypeError("expected BodyRates, PositionNed or VelocityNed")
        time_ms = int((time.monotonic() - self._boot) * 1000) & 0xffffffff
        if isinstance(command, BodyRates):
            self.mav.set_attitude_target_send(
                time_ms, *self.target_ids,
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
            self.mav.set_position_target_local_ned_send(
                time_ms, *self.target_ids, mavlink.MAV_FRAME_LOCAL_NED,
                mask, *position, *velocity, 0, 0, 0, 0, 0,
            )

    def heartbeat(self):
        self.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, mavlink.MAV_STATE_ACTIVE)

    def arm(self):
        self._set_armed(True)

    def disarm(self):
        self._set_armed(False)

    def _set_armed(self, armed):
        self.mav.command_long_send(*self.target_ids, mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    0, int(armed), 0, 0, 0, 0, 0, 0)


class _Track:
    """Handshake → indexed fragments → complete course; never expire it per tick."""

    GATE = struct.Struct("<H9f")
    CHUNK_BYTES = 250  # ENCAPSULATED_DATA minus its type and transfer ID.
    MAX_GATES = 1024

    def __init__(self):
        self.transfer = None
        self.chunks = {}
        self.gates = None
        self.received_at = None

    def start(self, message):
        size, packets = message.size, message.packets
        if (not 2 <= size <= 2 + self.MAX_GATES * self.GATE.size
                or packets != (size + self.CHUNK_BYTES - 1) // self.CHUNK_BYTES):
            return
        transfer = (message.width, size, packets)  # width carries the vendor's transfer ID.
        if transfer != self.transfer:
            self.transfer, self.chunks = transfer, {}

    def receive(self, message, now):
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
        self.gates, self.received_at = tuple(gates), now


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
