from collections import deque
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

from miniflight import (Attitude, BodyRates, Command, Frame, Motion, MotorOutputs,
                       Ned, PositionNed, State, VelocityNed)
from target import Target


@dataclass(frozen=True)
class Race:
    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int  # unchanged wire value
    received_at: float = 0.0  # host monotonic seconds


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
        self._race = None
        self._previous_time = None
        self._last_imu = None

    @property
    def connected(self):
        return self._peer is not None

    @property
    def race(self):
        """Latest race packet, even when read() is waiting for a new IMU sample."""
        return self._race

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
        self._race = self._previous_time = self._last_imu = None
        self._boot = time.monotonic()
        self._rx = mavlink.MAVLink(None)
        self._rx.robust_parsing = True
        self._mav = mavlink.MAVLink(self, srcSystem=255, srcComponent=191)
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
                for message in self._rx.parse_buffer(packet) or ():
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
            race = Race(*struct.unpack_from("<BQqqIq", bytes(message.data))[1:], received_at=now)
            if (self._race is not None and race.sim_boot_time_ms < self._race.sim_boot_time_ms
                    and all(r.race_start_boot_time_ms < 0 or r.sim_boot_time_ms < r.race_start_boot_time_ms
                            for r in (self._race, race))):
                # The native sensor clock can restart before GO.
                self._telemetry.pop("HIGHRES_IMU", None)
                self._received_at.pop("HIGHRES_IMU", None)
                self._last_imu = self._previous_time = None
            self._race = race

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
        dt = 0.0 if self._previous_time is None else stamp - self._previous_time
        self._previous_time, self._last_imu = stamp, imu
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

    def _ned(self, north, east, down, *, velocity):
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
