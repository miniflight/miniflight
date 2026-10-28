"""Replay AIGP transport boundaries; Vehicle still constructs the observations."""

import base64
import argparse
from contextlib import contextmanager, ExitStack
from dataclasses import asdict
import json
from pathlib import Path
import time

from pymavlink.dialects.v20 import common as mavlink

from target.aigp.client import Packet
from miniflight.position import PositionConfig
from miniflight.replay import Calls
from target.aigp.experiments.recording import RecordedClient, command_data, read_metadata
from target.aigp.native import NativeRaceState, launch, sha256


def packet_data(packet):
    camera = isinstance(packet.data, bytes)
    return dict(camera=camera, received_at=packet.received_at,
                wire=base64.b64encode(packet.data if camera else packet.data.get_msgbuf()).decode("ascii"))


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


class ProcessClient(RecordedClient):
    def __init__(self, calls, timesync=False, **kwargs):
        super().__init__(calls.record or (lambda **row: None), **kwargs)
        self.calls, self.decoder = calls, mavlink.MAVLink(None)
        self.outgoing, self.wire_commands = None, 0
        self.timesync = timesync

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
        raise ValueError("full replay needs a flight or reset process recording")
    outcome = {}
    def recorded_calls():
        outgoing, wire_recorded = [], False
        with path.open() as source:
            for row in map(json.loads, source):
                if row["event"] == "outgoing":
                    outgoing.append(row["wire"])
                    wire_recorded = True
                if row["event"] == "call":
                    if row["name"] == "send" and "error" not in row and wire_recorded:
                        if len(outgoing) != 1:
                            raise AssertionError("recorded send must contain one outgoing packet")
                        client.outgoing = outgoing[0]
                    outgoing.clear()
                    yield row
                elif row["event"] in ("result", "error"):
                    outcome.setdefault(row["event"], row)
    calls = Calls(rows=recorded_calls())
    client = ProcessClient(calls, camera_port=5600 if metadata.get("camera") else None, timesync=metadata.get("timesync", False))
    if metadata["process"] == "flight":
        reference = metadata["reference"]
        if reference is not None and sha256(Path(reference["path"])) != reference["sha256"]:
            raise ValueError("recorded reference changed")
        make, _, _ = flight_controller(metadata["controller"], None if reference is None else reference["path"],
                                       None if "controller_config" not in metadata else PositionConfig(**metadata["controller_config"]),
                                       metadata.get("trajectory_pid", {}).get("ki", 0.0))
        simulator = AIGPSimulator(make, metadata["target"], hz=metadata["hz"], timeout=metadata["timeout"],
                                  client=client, race_reader=RaceReader(calls), clock=calls, finish_before=metadata.get("finish_before_s"),
                                  race_status_path=metadata.get("race_status_path"), launcher=Launcher(calls),
                                  startup_timeout=metadata.get("startup_timeout_s", 120))
        execute = lambda: simulator.rollout(attach=metadata.get("attach", True), simulator_args=metadata.get("simulator_args", ()))
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
    expected_error = outcome.get("error", {}).get("error")
    expected = outcome["result"]
    result = (dict(race=None if simulator.status is None else asdict(simulator.status)) if metadata["process"] == "flight"
              else dict(reset=reset, course=[packet_data(packet) for packet in course]))
    result["connection_closed"] = not client.connected
    if "returned" in expected:
        result["returned"] = None if returned is None else asdict(returned)
    if error != expected_error or json.loads(json.dumps(result)) != {key: expected[key] for key in result}:
        raise AssertionError(f"process outcome differs: {error}, result={result}")
    return dict(trace=str(path), calls=calls.count, wire_commands=client.wire_commands, error=error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path, nargs="+")
    for path in parser.parse_args().traces:
        print(json.dumps(replay(path)), flush=True)


if __name__ == "__main__":
    main()
