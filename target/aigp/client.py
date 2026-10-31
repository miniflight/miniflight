"""Receive asynchronous AI-GP packets; read observations and encode commands."""

from dataclasses import dataclass, replace
import math
import select
import socket
import struct
import time
from types import MappingProxyType

import cv2
import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

from miniflight import Attitude, BodyRates, Command, Frame, Motion, MotorControls, MotorOutputs, Ned, PositionNed, State, VelocityNed
from miniflight.state import ImuSample
from target import Target
from target.aigp.controllers import Gate


@dataclass(frozen=True)
class RaceTelemetry:
    """Decoded MAVLink race fields; no inferred game state."""

    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int  # unchanged wire value
    received_at: float = 0.0  # host monotonic seconds


@dataclass(frozen=True)
class TrackInfo:
    """Fixed gate layout from one completed native transfer; it may be redacted."""

    transfer_id: int
    gates: tuple[Gate, ...]


@dataclass(frozen=True)
class Packet:
    """Original message or datagram, receipt time, and optional decoded observation."""

    data: mavlink.MAVLink_message | bytes
    received_at: float  # host monotonic receipt; device timestamps remain in data
    decoded: RaceTelemetry | TrackInfo | Frame | ImuSample | Motion | Attitude | None = None

    @property
    def privileged(self) -> bool:
        """AIGP pose or course traffic, regardless of whether its values are usable."""
        if isinstance(self.data, bytes):
            return False
        kind = self.data.get_type()
        return (kind in ("ATTITUDE", "LOCAL_POSITION_NED", "ODOMETRY", "DATA_TRANSMISSION_HANDSHAKE")
                or kind == "ENCAPSULATED_DATA" and self.data.data[0] == 2)


def _newer(previous, sample):
    if previous is None or (sample.time >= previous.time and replace(sample, received_at=previous.received_at) != previous):
        return sample
    return previous


def _imu_sample(imu):
    return ImuSample(imu.time_usec * 1e-6, imu._host_received_at,
                     (imu.xacc, imu.yacc, imu.zacc), (-imu.xgyro, -imu.ygyro, -imu.zgyro))


class SimulatorClient(Target):
    """Receive native packets and send control commands."""

    commands = frozenset((BodyRates, PositionNed, VelocityNed, MotorControls))

    def __init__(self, port=14550, camera_port=5600):
        self.port = port
        self.camera_port = camera_port
        self.connection = self._vision = self.target_ids = None
        self._telemetry = {}
        self.telemetry = MappingProxyType(self._telemetry)  # Message name → latest raw MAVLink packet.
        self._track = _Track()
        self._camera = _Camera()
        self.race_telemetry: RaceTelemetry | None = None

    @property
    def connected(self):
        return self.target_ids is not None

    @property
    def mav(self):
        if self.connection is None or not self.connected:
            raise RuntimeError("client is not connected")
        if self.connection.port.fileno() < 0:
            raise OSError("MAVLink connection is closed")
        return self.connection.mav

    @property
    def gates(self):
        """Latest complete native course, including redacted or unusable fields."""
        return self._track.gates

    @property
    def gates_received_at(self):
        """Host monotonic receipt of that complete transfer, or None. Does not poll."""
        return self._track.received_at

    def open(self):
        """Reserve the UDP ports without waiting for or commanding the simulator."""
        if self.connection is not None:
            raise RuntimeError("client is already open")
        self._telemetry.clear()
        self.state = None
        self._camera = _Camera()
        self._track = _Track()
        self.race_telemetry = None
        self._boot = time.monotonic()
        try:
            self.connection = mavutil.mavlink_connection(
                f"udpin:127.0.0.1:{self.port}", dialect="common", source_component=191)
            # Keep the simulator port exclusive to this controller.
            self.connection.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
            if self.camera_port is not None:
                self._vision = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._vision.bind(("127.0.0.1", self.camera_port))
                self._vision.setblocking(False)
        except BaseException:
            self.disconnect()
            raise

    def connect(self):
        self.open()

    def disconnect(self):
        for sock in (self.connection, self._vision):
            if sock is not None:
                sock.close()
        self.connection = self._vision = self.target_ids = None

    def read(self, timeout=1.0):
        self.packets = self.poll(timeout)
        previous = self.state or State()
        motion, attitude = previous.motion, previous.attitude
        for packet in self.packets:
            if isinstance(packet.decoded, Motion):
                motion = _newer(motion, packet.decoded)
            elif isinstance(packet.decoded, Attitude):
                attitude = _newer(attitude, packet.decoded)
        motors = self._telemetry.get("ACTUATOR_OUTPUT_STATUS")
        if motors is not None:
            motors = MotorOutputs(motors.time_usec * 1e-6, motors._host_received_at, tuple(motors.actuator), motors.active)
        samples = tuple(packet.decoded for packet in self.packets if isinstance(packet.decoded, ImuSample))
        self.state = State(frame=self._camera.latest, motion=motion, attitude=attitude, motors=motors, imu_samples=samples)
        imu = self._telemetry.get("HIGHRES_IMU")
        if imu is not None:
            sample = samples[-1] if samples else _imu_sample(imu)
            dt = 0.0 if previous.time is None else sample.time - previous.time
            self.state = replace(self.state, time=sample.time, dt=dt, received_at=sample.received_at,
                                 acceleration=sample.acceleration, gyro=sample.gyro)
        return self.state

    def poll(self, timeout=0.0) -> tuple[Packet, ...]:
        if self.connection is None:
            raise RuntimeError("client is not connected")
        arrivals = []
        sockets = [s for s in (self.connection.port, self._vision) if s is not None]
        # Bound each drain so continuous traffic cannot starve the controller.
        for _ in range(128):
            # Drain buffered messages too: one UDP datagram can contain several.
            message = self.connection.recv_msg()
            if message is not None:
                now = time.monotonic()
                decoded = self._receive(message, now)
                if (message.get_type() != "BAD_DATA" and self.target_ids is not None
                        and message.get_srcSystem() == self.target_ids[0]):
                    arrivals.append(Packet(message, now, decoded))
                timeout = 0.0
            ready, _, _ = select.select(sockets, [], [], timeout)
            if not ready and message is None:
                break
            timeout = 0.0
            if self._vision in ready:
                packet, _ = self._vision.recvfrom(65536)
                now = time.monotonic()
                arrivals.append(Packet(packet, now, self._camera.receive(packet, now)))
        return tuple(arrivals)

    def _receive(self, message, now):
        kind = message.get_type()
        if kind == "BAD_DATA":
            return
        source = (message.get_srcSystem(), message.get_srcComponent())
        if self.target_ids is None:
            if kind != "HEARTBEAT":
                return
            self.target_ids = source
            if self.connection is not None:
                self.connection.target_system, self.connection.target_component = source
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
        if kind == "HIGHRES_IMU":
            return _imu_sample(message)
        elif kind == "LOCAL_POSITION_NED":
            return Motion(message.time_boot_ms / 1000, now, Ned(message.x, message.y, message.z), Ned(message.vx, message.vy, message.vz))
        elif kind == "ATTITUDE":
            return Attitude(message.time_boot_ms / 1000, now, message.roll, -message.pitch, -message.yaw,
                            (-message.rollspeed, -message.pitchspeed, -message.yawspeed))
        elif kind == "DATA_TRANSMISSION_HANDSHAKE":
            # Announcement only: transfer ID, byte count, fragment count.
            self._track.start(message)
        elif kind == "ENCAPSULATED_DATA" and message.data[0] == 2:
            # Fragment arrival may complete a track; sensor arrival never does.
            return self._track.receive(message, now)
        elif kind == "ENCAPSULATED_DATA" and message.data[0] == 1:
            # Native boot/start/finish/gate values, not an IMU or host tick.
            race = RaceTelemetry(*struct.unpack_from("<BQqqIq", bytes(message.data))[1:], received_at=now)
            if self.race_telemetry is not None and race.sim_boot_time_ms < self.race_telemetry.sim_boot_time_ms:
                # A restarted source clock cannot inherit the previous IMU watermark.
                self._telemetry.pop("HIGHRES_IMU", None)
            self.race_telemetry = race
            return race

    def send(self, command: Command):
        self.mav.send(self.encode_command(command, time.monotonic() - self._boot, self.target_ids, self.mav))

    @staticmethod
    def encode_command(command: Command, elapsed, target_ids, encoder):
        """Construct the native message without sending it."""
        if not isinstance(command, Command):
            raise TypeError("expected a flight command")
        time_ms = int(elapsed * 1000) & 0xffffffff
        if isinstance(command, BodyRates):
            return encoder.set_attitude_target_encode(
                time_ms, *target_ids,
                mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE | 16,  # AI-GP rad/s extension
                [1.0, 0.0, 0.0, 0.0],
                # The simulator's angular-rate wire axes are opposite to FRD.
                -command.roll_rate, -command.pitch_rate, -command.yaw_rate, command.thrust,
            )
        elif isinstance(command, MotorControls):
            if len(command.outputs) != 4:
                raise ValueError("AIGP requires four motor controls")
            return encoder.set_actuator_control_target_encode(
                int(elapsed * 1e6), 0, *target_ids, command.outputs + (0.0,) * 4)
        else:
            vector = (command.north, command.east, command.down)
            position = vector if isinstance(command, PositionNed) else (0, 0, 0)
            velocity = vector if isinstance(command, VelocityNed) else (0, 0, 0)
            # 1 ignores a field: xyz bits 0..2, velocity 3..5, acceleration 6..8, yaw 10..11.
            mask = 0b110111111000 if isinstance(command, PositionNed) else 0b110111000111
            return encoder.set_position_target_local_ned_encode(
                time_ms, *target_ids, mavlink.MAV_FRAME_LOCAL_NED,
                mask, *position, *velocity, 0, 0, 0, 0, 0,
            )

    def heartbeat(self):
        self.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, mavlink.MAV_STATE_ACTIVE)

    def arm(self):
        self._set_armed(True)

    def disarm(self):
        self._set_armed(False)

    def reset(self):
        """Request native reset; the caller must observe a fresh race afterward."""
        self.mav.command_long_send(*self.target_ids, 31000, 0, 0, 0, 0, 0, 0, 0, 0)

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
            self.transfer = transfer
            self.chunks = {}

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
        self.chunks[index] = payload[3:]
        if len(self.chunks) != packets:
            return
        data = b"".join(self.chunks[i] for i in range(packets))[:size]
        self.transfer, self.chunks = None, {}
        count, = struct.unpack_from("<H", data)
        if not 0 <= count <= self.MAX_GATES or len(data) != 2 + count * self.GATE.size:
            return
        gates = []
        for row in self.GATE.iter_unpack(data[2:]):
            gate_id, north, east, down, w, x, y, z, width, height = row
            gates.append(Gate(gate_id, Ned(north, east, down), (w, x, y, z), width, height))
        gates = tuple(gates)
        self.gates, self.received_at = gates, now
        return TrackInfo(transfer_id, gates)


class _Camera:
    """Assemble the simulator's UDP JPEG packets; discard incomplete frames."""

    HEADER = struct.Struct("<IHHIIQ")
    MAX_BYTES = 8 * 1024 * 1024
    MAX_AGE = 0.5

    def __init__(self):
        self.frame = None
        self.started_at = 0.0
        self.chunks = {}
        self.latest = self.jpeg = None

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
            self.jpeg = jpeg
            self.latest = Frame(frame_id, timestamp, now, bgr)
            return self.latest
