"""Replay AIGP transport boundaries; Vehicle still constructs the observations."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from miniflight.position import PositionConfig
from miniflight.replay import Calls
from target.aigp.experiments.recording import Launcher, ProcessClient, RaceReader, packet_data, read_metadata
from target.aigp.native import sha256


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
