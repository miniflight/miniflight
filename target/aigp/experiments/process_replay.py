"""Replay AIGP transport boundaries; Vehicle still constructs the observations."""

import base64
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.client import Packet
from miniflight.position import PositionConfig
from miniflight.replay import Calls
from target.aigp.experiments.recording import RecordedClient, command_data, read_metadata
from target.aigp.native import NativeRaceState, sha256


def packet_data(packet):
    camera = isinstance(packet.data, bytes)
    return dict(camera=camera, received_at=packet.received_at,
                wire=base64.b64encode(packet.data if camera else packet.data.get_msgbuf()).decode("ascii"))


class ProcessClient(RecordedClient):
    def __init__(self, calls, **kwargs):
        super().__init__(calls.record or (lambda **row: None), **kwargs)
        self.calls, self.decoder = calls, mavlink.MAVLink(None)

    def open(self):
        return self.calls("open", dict(port=self.port, camera_port=self.camera_port), super().open)

    def disconnect(self):
        self.calls("disconnect", {}, super().disconnect)
        self.target_ids = None

    def heartbeat(self):
        return self.calls("heartbeat", {}, super().heartbeat)

    def reset(self):
        return self.calls("reset", {}, super().reset)

    def _set_armed(self, armed):
        return self.calls("arm", dict(armed=armed), lambda: super(ProcessClient, self)._set_armed(armed))

    def send(self, command):
        return self.calls("send", command_data(command), lambda: super(ProcessClient, self).send(command))

    def poll(self, timeout=0.0):
        packets = self.calls("poll", dict(timeout=timeout), lambda: super(ProcessClient, self).poll(timeout),
                             encode=lambda packets: [packet_data(packet) for packet in packets])
        if self.calls.record is not None:
            return packets
        return tuple(self.receive(packet) for packet in packets)

    def receive(self, packet):
        wire, stamp = base64.b64decode(packet["wire"], validate=True), packet["received_at"]
        if packet["camera"]:
            return Packet(wire, stamp, self._camera.receive(wire, stamp))
        message, = self.decoder.parse_buffer(wire)
        return Packet(message, stamp, self._receive(message, stamp))


class RaceReader:
    def __init__(self, calls, reader=None):
        self.calls, self.reader = calls, reader

    def poll(self):
        states = self.calls("race", {}, None if self.reader is None else self.reader.poll,
                            encode=lambda states: [asdict(state) for state in states])
        return states if self.reader is not None else tuple(NativeRaceState(**state) for state in states)


def replay(path):
    from target.aigp.experiments.flight import flight_controller
    from target.aigp.experiments.repeat import fresh_race
    from target.aigp.simulator import AIGPSimulator
    path = Path(path)
    metadata = read_metadata(path)
    if metadata.get("process") not in ("flight", "reset"):
        raise ValueError("full replay needs a process recording from an attached flight or reset")
    with path.open() as source:
        rows = [json.loads(line) for line in source]
    calls = Calls(rows=(row for row in rows if row["event"] == "call"))
    client = ProcessClient(calls, camera_port=5600 if metadata.get("camera") else None)
    if metadata["process"] == "flight":
        reference = metadata["reference"]
        if reference is not None and sha256(Path(reference["path"])) != reference["sha256"]:
            raise ValueError("recorded reference changed")
        make, _, _ = flight_controller(metadata["controller"], None if reference is None else reference["path"],
                                       None if "controller_config" not in metadata else PositionConfig(**metadata["controller_config"]),
                                       metadata.get("trajectory_pid", {}).get("ki", 0.0))
        simulator = AIGPSimulator(make, metadata["target"], hz=metadata["hz"], timeout=metadata["timeout"],
                                  client=client, race_reader=RaceReader(calls), clock=calls)
        execute = lambda: simulator.rollout(attach=True)
    else:
        reset, course = None, tuple(client.receive(packet) for packet in metadata["course"])
        def execute():
            nonlocal reset, course
            _, reset, course = fresh_race(RaceReader(calls), lambda **row: None, course_packets=course, client=client, clock=calls)
    error, returned = None, None
    try:
        returned = execute()
    except BaseException as exception:
        error = dict(type=type(exception).__name__, message=str(exception))
    calls.finish()
    expected_error = next((row["error"] for row in rows if row["event"] == "error"), None)
    expected = next(row for row in rows if row["event"] == "result")
    result = (dict(race=None if simulator.status is None else asdict(simulator.status)) if metadata["process"] == "flight"
              else dict(reset=reset, course=[packet_data(packet) for packet in course]))
    result["connection_closed"] = not client.connected
    if "returned" in expected:
        result["returned"] = None if returned is None else asdict(returned)
    if error != expected_error or json.loads(json.dumps(result)) != {key: expected[key] for key in result}:
        raise AssertionError(f"process outcome differs: {error}, result={result}")
    return dict(trace=str(path), calls=calls.count, error=error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path, nargs="+")
    for path in parser.parse_args().traces:
        print(json.dumps(replay(path)), flush=True)


if __name__ == "__main__":
    main()
