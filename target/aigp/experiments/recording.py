"""Record controller inputs and outputs; replay them without a simulator."""

import base64
from contextlib import contextmanager
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

import cv2
import numpy as np

from miniflight import Attitude, Frame, Motion, MotorOutputs, Ned, State
from target.aigp.controllers import BaseController, CommandT, Gate
from target.aigp.controllers.r1_gates import gate_center
from target.aigp.simulator import SimulatorClient, TARGETS


def command_data(command):
    return None if command is None else {"kind": type(command).__name__, **asdict(command)}


def state_data(state, frames=True):
    data = asdict(replace(state, frame=None))
    if frames and state.frame is not None:
        frame = state.frame
        if frame.bgr.dtype != np.uint8 or frame.bgr.ndim != 3 or frame.bgr.shape[2] != 3:
            raise ValueError("recorded frames must be uint8 BGR images")
        success, png = cv2.imencode(".png", frame.bgr)
        if not success:
            raise ValueError("could not encode the recorded frame")
        data["frame"] = dict(id=frame.id, time_ns=frame.time_ns, received_at=frame.received_at,
                             png=base64.b64encode(png).decode("ascii"))
    return data


def read_state(data):
    data = dict(data)
    data["acceleration"], data["gyro"] = tuple(data["acceleration"]), tuple(data["gyro"])
    if data["motion"] is not None:
        motion = dict(data["motion"])
        motion["position"], motion["velocity"] = Ned(*motion["position"]), Ned(*motion["velocity"])
        data["motion"] = Motion(**motion)
    if data["attitude"] is not None:
        data["attitude"] = Attitude(**data["attitude"])
    if data["motors"] is not None:
        motors = dict(data["motors"])
        motors["outputs"] = tuple(motors["outputs"])
        data["motors"] = MotorOutputs(**motors)
    if data["frame"] is not None:
        frame = dict(data["frame"])
        png = base64.b64decode(frame.pop("png"), validate=True)
        bgr = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("invalid recorded frame")
        bgr.flags.writeable = False
        data["frame"] = Frame(**frame, bgr=bgr)
    return State(**data)


def read_gate(data):
    data = dict(data)
    data["orientation"] = tuple(data["orientation"])
    if "position" not in data:
        # Snapshot recordings stored a derived center, sometimes also the native base.
        origin = data.pop("origin", None)
        center = data.pop("center")
        offset = gate_center(Gate(data["id"], Ned(0, 0, 0), data["orientation"], data["width"], data["height"]))
        data["position"] = origin if origin is not None else tuple(c - d for c, d in zip(center, offset))
    data["position"] = Ned(*data["position"])
    return Gate(**data)


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot record {type(value).__name__}")


@contextmanager
def recording(path, metadata=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", buffering=1) as output:
        def record(**row):
            output.write(json.dumps(dict(host_time=time.monotonic(), **row),
                                    default=_json_default, allow_nan=False) + "\n")

        record(event="config", format=1, metadata={} if metadata is None else metadata)
        yield record


def _read_header(source):
    line = next(source, None)
    if line is None:
        raise ValueError("empty recording")
    header = json.loads(line)
    if not isinstance(header, dict) or header.get("event") != "config" or header.get("format", 0) not in (0, 1):
        raise ValueError("unsupported recording format")
    return header


def read_metadata(path):
    with Path(path).open() as source:
        header = _read_header(source)
    return header["metadata"] if header.get("format") == 1 else header


class RecordedController(BaseController[CommandT]):
    def __init__(self, controller: BaseController[CommandT], record, frames=True):
        self.controller, self.record, self.frames = controller, record, frames

    @property
    def targets(self):
        return getattr(self.controller, "targets", TARGETS)

    def update(self, state, gate_index, gates) -> CommandT | None:
        row = dict(event="update", state=state_data(state, self.frames), gate_index=gate_index,
                   gates=None if gates is None else [asdict(gate) for gate in gates])
        if not self.frames and state.frame is not None:
            frame = state.frame
            row["frame_info"] = dict(id=frame.id, time_ns=frame.time_ns, received_at=frame.received_at,
                                     shape=frame.bgr.shape, dtype=frame.bgr.dtype.name)
        try:
            command = self.controller.update(state, gate_index, gates)
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
        super()._receive(message, peer, now)
        if (message.get_type() in ("COMMAND_ACK", "COLLISION", "HEARTBEAT")
                and self._telemetry.get(message.get_type()) is message):
            self.record(event="telemetry", message=message.to_dict())


def replay(controller, path):
    """Return the number of matching updates. The caller supplies a fresh controller."""
    updates, terminal = 0, False
    with Path(path).open() as source:
        header = _read_header(source)
        for line in source:
            row = json.loads(line)
            if row["event"] != "update":
                continue
            if terminal:
                raise ValueError("recording contains updates after a terminal exception")
            state = read_state(row["state"])
            # Older probe traces did not record geometry; their controller did not use it.
            data = row["gates"] if header.get("format") == 1 else row.get("gates")
            gates = None if data is None else tuple(read_gate(gate) for gate in data)
            try:
                command = controller.update(state, row["gate_index"], gates)
            except BaseException as error:
                terminal = True
                actual = {"error": {"type": type(error).__name__, "message": str(error)}}
                if isinstance(row.get("error"), str):
                    actual = {"error": type(error).__name__}
            else:
                actual = {"command": command_data(command)}
            expected = {key: row[key] for key in ("command", "error") if key in row}
            if actual != expected:
                raise AssertionError(f"replay differs at update {updates}: expected {expected}, got {actual}")
            updates += 1
    if not updates:
        raise ValueError("recording contains no controller updates")
    return updates
