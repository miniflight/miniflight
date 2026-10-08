"""Record native arrivals and commands; replay the same controller calls."""

import base64
from contextlib import contextmanager
from dataclasses import asdict
import json
from pathlib import Path
from threading import Lock
import time

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from miniflight import Frame
from target.aigp.client import Packet, SimulatorClient
from target.aigp.controllers import BaseController, CommandT
from target.aigp.native import TARGETS, NativeRaceState


def command_data(command):
    return None if command is None else {"kind": type(command).__name__, **asdict(command)}


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
        yield record


def _read_header(source):
    header = json.loads(next(source, "null"))
    if not isinstance(header, dict) or header.get("event") != "config" or header.get("format", 0) not in (0, 1, 2, 3):
        raise ValueError("unsupported recording format")
    return header


def read_metadata(path):
    with Path(path).open() as source:
        header = _read_header(source)
    return header["metadata"] if header.get("format") in (1, 2, 3) else header


class RecordedController(BaseController[CommandT]):
    def __init__(self, create, record, frames=False):
        self.create, self.record, self.frames = create, record, frames
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
        self.record(**row, command=command_data(command))
        return command


class RecordedClient(SimulatorClient):
    def __init__(self, record, port=14550, camera_port=5600):
        super().__init__(port=port, camera_port=camera_port)
        self.record = record

    def send(self, command):
        super().send(command)
        self.record(event="sent", command=command_data(command))

    def _set_armed(self, armed):
        super()._set_armed(armed)
        self.record(event="arm_request", armed=armed)

    def _receive(self, message, peer, now):
        decoded = super()._receive(message, peer, now)
        if (message.get_type() in ("COMMAND_ACK", "COLLISION", "HEARTBEAT")
                and self._telemetry.get(message.get_type()) is message):
            self.record(event="telemetry", message=message.to_dict())
        return decoded


def replay(create, path):
    """Replay native-arrival recordings. Older State snapshots use their old API."""
    updates, terminal, controller = 0, False, None
    decoder, client = mavlink.MAVLink(None), SimulatorClient(camera_port=None)
    with Path(path).open() as source:
        if _read_header(source).get("format") != 3:
            raise ValueError("older recordings lack native race state; use their recorded controller API")
        for line in source:
            row = json.loads(line)
            if row["event"] == "init":
                controller = create()
            if row["event"] != "update":
                continue
            if terminal or controller is None:
                raise ValueError("controller updates outside its lifetime")
            telemetry = []
            for packet in row["telemetry"]:
                message, = decoder.parse_buffer(base64.b64decode(packet["wire"], validate=True))
                stamp = packet["received_at"]
                telemetry.append(Packet(message, stamp, client._receive(message, ("replay", 0), stamp)))
            frames = tuple(frame for data in row["frames"] if (frame := read_frame(data)) is not None)
            try:
                race = row["native_race"]
                command = controller.update(tuple(telemetry), frames, None if race is None else NativeRaceState(**race))
            except BaseException as error:
                terminal = True
                actual = {"error": {"type": type(error).__name__, "message": str(error)}}
            else:
                actual = {"command": command_data(command)}
            expected = {key: row[key] for key in ("command", "error") if key in row}
            if actual != expected:
                raise AssertionError(f"replay differs at update {updates}: expected {expected}, got {actual}")
            updates += 1
    if not updates:
        raise ValueError("recording contains no controller updates")
    return updates
