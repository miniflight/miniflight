"""Record native arrivals and commands; replay the same controller calls."""

import base64
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
import json
from pathlib import Path
from threading import Lock
import time

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from miniflight import Frame, vehicle
from target.aigp.client import Packet, SimulatorClient
from target.aigp.controllers import BaseController, CommandT, update_controller
from target.aigp.native import TARGETS, NativeRaceState, launch


def command_data(command):
    return None if command is None else {"kind": type(command).__name__,
        **{key: list(value) if isinstance(value, tuple) else value for key, value in asdict(command).items()}}


def packet_data(packet):
    camera = isinstance(packet.data, bytes)
    return dict(camera=camera, received_at=packet.received_at,
                wire=base64.b64encode(packet.data if camera else packet.data.get_msgbuf()).decode("ascii"))


def frame_data(frame, pixels=False):
    data = dict(id=frame.id, time_ns=frame.time_ns, received_at=frame.received_at, shape=frame.bgr.shape)
    if pixels:
        success, png = cv2.imencode(".png", frame.bgr)
        if not success:
            raise ValueError("could not encode the recorded frame")
        data["png"] = base64.b64encode(png).decode("ascii")
    return data


def read_frame(data):
    """Metadata-only recordings omit pixels; pixel-dependent replay needs PNGs."""
    if "png" not in data:
        return None
    png = base64.b64decode(data["png"], validate=True)
    bgr = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("invalid recorded frame")
    bgr.flags.writeable = False
    return Frame(data["id"], data["time_ns"], data["received_at"], bgr)


@contextmanager
def recording(path, metadata=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", buffering=1) as output:
        lock = Lock()
        def record(**row):
            line = json.dumps(dict(host_time=time.monotonic(), **row)) + "\n"
            with lock:
                output.write(line)

        record(event="config", format=3, metadata={} if metadata is None else metadata)
        try:
            yield record
        except BaseException as error:
            record(event="error", error={"type": type(error).__name__, "message": str(error)})
            raise


def _read_header(source):
    header = json.loads(next(source, "null"))
    if not isinstance(header, dict) or header.get("event") != "config" or header.get("format", 0) not in (0, 1, 2, 3):
        raise ValueError("unsupported recording format")
    return header


def read_metadata(path):
    with Path(path).open() as source:
        header = _read_header(source)
    return header["metadata"] if header.get("format") in (1, 2, 3) else header


def read_course(path):
    """Recover the immutable native course transfer from a recorded input batch."""
    decoder = mavlink.MAVLink(None)
    with Path(path).open() as source:
        _read_header(source)
        telemetry = next(row["telemetry"] for row in map(json.loads, source) if row["event"] == "update")
    packets = []
    for data in telemetry:
        message, = decoder.parse_buffer(base64.b64decode(data["wire"], validate=True))
        if message.get_type() == "DATA_TRANSMISSION_HANDSHAKE" or message.get_type() == "ENCAPSULATED_DATA" and message.data[0] == 2:
            packets.append(Packet(message, data["received_at"]))
    return tuple(packets)


class NativeProcess:
    def __init__(self, calls, process=None):
        self.calls, self.process = calls, process

    def poll(self):
        return self.calls("process.poll", {}, None if self.process is None else self.process.poll)


class Launcher:
    def __init__(self, calls):
        self.calls = calls

    @contextmanager
    def __call__(self, target, simulator_args=(), attach=False, race_status_path=None):
        if attach:
            with launch(target, simulator_args, attach=True, race_status_path=race_status_path) as process:
                yield process
            return
        cleanup = ExitStack()
        arguments = dict(target=target, simulator_args=list(simulator_args), race_status_path=str(race_status_path))
        try:
            process = self.calls("launch", arguments,
                                 lambda: cleanup.enter_context(launch(target, simulator_args, race_status_path=race_status_path)),
                                 encode=lambda process: process.pid)
            yield NativeProcess(self.calls, process if self.calls.record is not None else None)
        finally:
            self.calls("launch.close", {}, cleanup.close)


class RaceReader:
    def __init__(self, calls, reader=None):
        self.calls, self.reader = calls, reader

    def poll(self):
        states = self.calls("race", {}, None if self.reader is None else self.reader.poll,
                            encode=lambda states: [asdict(state) for state in states])
        return states if self.reader is not None else tuple(NativeRaceState(**state) for state in states)


class RecordedController(BaseController[CommandT]):
    def __init__(self, create, record, frames=False, signals=None):
        self.create, self.record, self.frames = create, record, frames
        self.signals = signals
        self.controller = None

    @property
    def targets(self):
        return getattr(self.create, "targets", TARGETS)

    def __call__(self):
        self.controller = self.create()
        self.record(event="init")
        return self

    def update(self, telemetry, frames, race_state) -> CommandT | None:
        row = dict(event="update", telemetry=[dict(received_at=packet.received_at,
                   wire=base64.b64encode(packet.data.get_msgbuf()).decode("ascii")) for packet in telemetry],
                   frames=[frame_data(frame, self.frames) for frame in frames],
                   native_race=None if race_state is None else asdict(race_state))
        try:
            command = self.controller.update(telemetry, frames, race_state)
        except BaseException as error:
            self.record(**row, error={"type": type(error).__name__, "message": str(error)})
            raise
        if command is not None and self.signals is not None:
            row["signals"] = self.signals(self.controller)
        self.record(**row, command=command_data(command))
        return command


class RecordedClient(SimulatorClient):
    def __init__(self, record, port=14550, camera_port=5600, course_packets=(), preview=None):
        super().__init__(port=port, camera_port=camera_port)
        self.record = record
        self.course_packets = course_packets  # Immutable course traffic retains its original receipt times.
        self.preview = None if preview is None else Path(preview)

    def open(self):
        super().open()
        receive = self.connection.recv

        def receive_datagram(n=None):
            data = receive(n)
            if data:
                self.record(event="mavlink_datagram", wire=base64.b64encode(data).decode("ascii"))
            return data

        self.connection.recv = receive_datagram
        self.connection.mav.set_send_callback(lambda message: self.record(
            event="outgoing", wire=base64.b64encode(message.get_msgbuf()).decode("ascii")))

    def poll(self, timeout=0.0):
        packets = super().poll(timeout)
        if self.preview is not None and any(isinstance(packet.decoded, Frame) for packet in packets):
            temporary = self.preview.with_suffix(".tmp")
            temporary.write_bytes(self._camera.jpeg)
            temporary.replace(self.preview)
        if self.connected and self.course_packets:
            course = tuple(Packet(packet.data, packet.received_at, self._receive(packet.data, packet.received_at))
                           for packet in self.course_packets)
            self.course_packets = ()
            return packets + course
        return packets

    def send(self, command):
        super().send(command)
        self.record(event="sent", command=command_data(command))

    def _set_armed(self, armed):
        super()._set_armed(armed)
        self.record(event="arm_request", armed=armed)

    def reset(self):
        super().reset()
        self.record(event="reset_request")

    def _receive(self, message, now):
        decoded = super()._receive(message, now)
        kind = message.get_type()
        if ((kind in ("COMMAND_ACK", "COLLISION", "HEARTBEAT") or kind == "ENCAPSULATED_DATA" and message.data[0] == 1)
                and self._telemetry.get(kind) is message):
            self.record(event="telemetry", message=message.to_dict())
        return decoded


class ProcessClient(RecordedClient):
    def __init__(self, calls, timesync=False, record_camera=True, **kwargs):
        super().__init__(calls.record or (lambda **row: None), **kwargs)
        self.calls, self.decoder = calls, mavlink.MAVLink(None)
        self.outgoing, self.wire_commands = None, 0
        self.timesync = timesync
        self.record_camera = record_camera

    def open(self):
        return self.calls("open", dict(port=self.port, camera_port=self.camera_port), super().open)

    def disconnect(self):
        self.calls("disconnect", {}, super().disconnect)
        self.target_ids = None

    def heartbeat(self):
        result = self.calls("heartbeat", {}, super().heartbeat)
        if self.timesync:
            epoch = self.calls("epoch_ns", {}, time.time_ns)
            self.calls("timesync", dict(tc1=epoch, ts1=0), lambda: self.mav.timesync_send(epoch, 0))
        return result

    def reset(self):
        return self.calls("reset", {}, super().reset)

    def _set_armed(self, armed):
        return self.calls("arm", dict(armed=armed), lambda: super(ProcessClient, self)._set_armed(armed))

    def send(self, command):
        result = self.calls("send", command_data(command), lambda: super(ProcessClient, self).send(command))
        if self.outgoing is not None:
            expected, = self.decoder.parse_buffer(base64.b64decode(self.outgoing, validate=True))
            message = self.encode_command(command, 0, self.target_ids, self.decoder)
            actual, = self.decoder.parse_buffer(message.pack(self.decoder))
            # Transport clock and MAVLink envelope are outside command-content replay.
            contents = [{key: value for key, value in packet.to_dict().items() if key not in ("time_boot_ms", "time_usec")}
                        for packet in (expected, actual)]
            if contents[0] != contents[1]:
                self.calls.mismatch = f"wire command {self.wire_commands}: expected {contents[0]}, got {contents[1]}"
                raise AssertionError(self.calls.mismatch)
            self.outgoing, self.wire_commands = None, self.wire_commands + 1
        return result

    def poll(self, timeout=0.0):
        packets = self.calls("poll", dict(timeout=timeout), lambda: super(ProcessClient, self).poll(timeout),
                             encode=lambda packets: [packet_data(packet) for packet in packets
                                                     if self.record_camera or not isinstance(packet.data, bytes)])
        if self.calls.record is not None:
            return packets
        return tuple(self.receive(packet) for packet in packets)

    def receive(self, packet):
        wire, stamp = base64.b64decode(packet["wire"], validate=True), packet["received_at"]
        if packet["camera"]:
            return Packet(wire, stamp, self._camera.receive(wire, stamp))
        message, = self.decoder.parse_buffer(wire)
        return Packet(message, stamp, self._receive(message, stamp))


class ReplayClient(SimulatorClient):
    """Feed recorded packets through the target's observation path without sockets."""

    def __init__(self):
        super().__init__(camera_port=None)
        self.decoder = mavlink.MAVLink(None)

    def feed(self, telemetry):
        packets = []
        for packet in telemetry:
            wire = base64.b64decode(packet["wire"], validate=True)
            stamp = packet["received_at"]
            if packet.get("camera", False):
                packets.append(Packet(wire, stamp, self._camera.receive(wire, stamp)))
            else:
                message, = self.decoder.parse_buffer(wire)
                packets.append(Packet(message, stamp, self._receive(message, stamp)))
        self.packets = tuple(packets)
        return self.packets

    def poll(self, timeout=0.0):
        return self.packets


def replay(create, path, signals=None):
    """Replay input batches through Vehicle and controller; runtime lifecycle is not replayed."""
    updates, terminal, controller = 0, False, None
    client = ReplayClient()
    with Path(path).open() as source, ExitStack() as cleanup:
        if _read_header(source).get("format") != 3:
            raise ValueError("older recordings lack native race state; use their recorded controller API")
        vehicle.bind(client)
        cleanup.callback(vehicle.disconnect)
        for line in source:
            row = json.loads(line)
            if row["event"] == "init":
                controller = create()
            if row["event"] != "update":
                continue
            if terminal or controller is None:
                raise ValueError("controller updates outside its lifetime")
            telemetry = client.feed(row["telemetry"])
            frames = tuple(frame for data in row["frames"] if (frame := read_frame(data)) is not None)
            if frames:
                client._camera.latest = frames[-1]
            vehicle.read(timeout=0)
            try:
                race = row["native_race"]
                command = update_controller(controller, telemetry, frames, None if race is None else NativeRaceState(**race))
            except BaseException as error:
                terminal = True
                actual = {"error": {"type": type(error).__name__, "message": str(error)}}
            else:
                actual = {"command": command_data(command)}
            expected = {key: row[key] for key in ("command", "error") if key in row}
            if actual != expected:
                raise AssertionError(f"replay differs at update {updates}: expected {expected}, got {actual}")
            if signals is not None and "signals" in row and signals(controller) != row["signals"]:
                raise AssertionError(f"control signals differ at update {updates}")
            updates += 1
    if not updates:
        raise ValueError("recording contains no controller updates")
    return updates
