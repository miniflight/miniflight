"""Run one complete native VQ1 R1 flight and replay its recorded commands."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import subprocess
import time

from target.aigp.controllers.r1_body_rates import Controller as BodyRateGates
from target.aigp.controllers.r1_gates import Controller as PositionGates
from target.aigp.experiments.recording import RecordedClient, RecordedController, recording, replay
from target.aigp.simulator import AIGPSimulator, BASE, SHIPPING, sha256


CONTROLLERS = {"r1_gates": PositionGates, "r1_body_rates": BodyRateGates}
ROOT = BASE.parents[1]


def run(name, trace):
    make = CONTROLLERS[name]
    sources = ("miniflight/position.py", "miniflight/state.py", "miniflight/control.py",
               "miniflight/vehicle.py", "miniflight/__init__.py", "common/math.py", "target/__init__.py",
               "target/aigp/controllers/r1_gates.py", "target/aigp/controllers/r1_body_rates.py",
               "target/aigp/controllers/__init__.py", "target/aigp/simulator.py",
               "target/aigp/experiments/recording.py", "test/aigp_regression.py")
    metadata = dict(target="vq1.r1", controller=name, hz=50, timeout=1.0,
                    revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    sources={path: sha256(ROOT / path) for path in sources})
    result = None
    with recording(trace, metadata) as record:
        client = RecordedClient(record, camera_port=None)
        simulator = AIGPSimulator(RecordedController(make(), record), "vq1.r1", client=client)
        try:
            result = simulator.rollout()
        finally:
            record(event="result", connection_closed=not client.connected,
                   race=None if result is None else asdict(result),
                   executable_sha256=sha256(BASE / ".runtime/vq1" / SHIPPING))

    if result is None or not result.started or not result.finished or result.active_gate_index != 6:
        raise AssertionError("native R1 did not finish all six gates")
    if client.connected or client.gates is None or len(client.gates) != 6:
        raise AssertionError("native R1 did not expose six gates and close its connection")
    updates = replay(make(), trace)
    with trace.open() as source:
        rows = [json.loads(line) for line in source]
    if [row["armed"] for row in rows if row["event"] == "arm_request"] != [True, False]:
        raise AssertionError("native R1 did not arm and disarm once")
    if any(row["event"] == "telemetry" and row["message"]["mavpackettype"] == "COLLISION" for row in rows):
        raise AssertionError("native R1 reported a collision")
    print(json.dumps(dict(controller=name, gates=6, replayed_updates=updates, trace=str(trace))), flush=True)


def stop(signum, frame):
    raise TimeoutError("native regression exceeded 210 wall-clock seconds or was interrupted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controller", choices=CONTROLLERS)
    parser.add_argument("trace", type=Path, nargs="?")
    args = parser.parse_args()
    trace = args.trace or BASE / ".runtime/regressions" / f"{args.controller}-{time.time_ns()}.jsonl"
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGALRM):
        signal.signal(signum, stop)
    signal.alarm(210)
    try:
        run(args.controller, trace)
    finally:
        signal.alarm(0)


if __name__ == "__main__":
    main()
