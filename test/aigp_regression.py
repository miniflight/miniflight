"""Run one complete native VQ1 R1 flight and replay its recorded commands."""

import argparse
from dataclasses import asdict
from functools import partial
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np

from miniflight import BodyRates
from miniflight.reference import sample_motion
from target.aigp.controllers.r1_body_rates import Controller as BodyRateGates
from target.aigp.controllers.r1_gates import Controller as PositionGates
from target.aigp.controllers.r1_beautiful import Controller as PIDGates
from target.aigp.controllers.r1_motors import Controller as MotorGates
from target.aigp.experiments.recording import RecordedClient, RecordedController, command_data, recording, replay
from target.aigp.experiments.r1_signals import motor_signals, r1_signals
from target.aigp.native import BASE, SHIPPING, sha256
from target.aigp.simulator import AIGPSimulator


CONTROLLERS = {"r1_gates": PositionGates, "r1_body_rates": BodyRateGates, "r1_beautiful": PIDGates, "r1_motors": MotorGates}
ROOT = BASE.parents[1]


def run(name, trace, camera=False, record_frames=False, signals=None, reference=None, offscreen=False):
    camera = camera or record_frames
    simulator_args = ("-RenderOffScreen",) if offscreen else ()
    make = CONTROLLERS[name]
    if reference is not None:
        if name != "r1_motors":
            raise ValueError("a recorded reference requires r1_motors")
        reference = Path(reference)
        with np.load(reference) as data:
            times = data["times"] if "times" in data else None
            trajectory = partial(sample_motion, data["states"], data["acceleration_rates"], float(data["dt"]), times=times)
        make = lambda: MotorGates(position=PIDGates(reference=trajectory))
        if signals is None:
            signals = lambda controller: {**r1_signals(controller.position), **motor_signals(controller)}
    timeout = .3 if name == "r1_motors" else 1.0
    if name == "r1_motors" and signals is None:
        signals = motor_signals
    sources = ("miniflight/rate.py", "miniflight/mixer.py", "target/aigp/__init__.py", "target/aigp/controllers/r1_motors.py",
               "miniflight/position.py", "miniflight/reference.py", "miniflight/state.py", "miniflight/control.py", "miniflight/monitor.py",
               "miniflight/vehicle.py", "miniflight/__init__.py", "common/math.py", "target/__init__.py",
               "target/aigp/controllers/r1_gates.py", "target/aigp/controllers/r1_body_rates.py", "target/aigp/controllers/r1_beautiful.py",
               "target/aigp/controllers/__init__.py", "target/aigp/simulator.py",
               "target/aigp/client.py", "target/aigp/native.py",
               "target/aigp/config/vq1/main.lua", "target/aigp/config/race.lua",
               "target/aigp/experiments/recording.py", "target/aigp/experiments/r1_signals.py", "test/aigp_regression.py")
    metadata = dict(target="vq1.r1", controller=name, hz=50, timeout=timeout, camera=camera, recorded_frames=record_frames,
                    simulator_args=simulator_args,
                    reference=None if reference is None else dict(path=str(reference.resolve()), sha256=sha256(reference)),
                    revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    sources={path: sha256(ROOT / path) for path in sources})
    result = None
    with recording(trace, metadata) as record:
        client = RecordedClient(record, camera_port=5600 if camera else None)
        simulator = AIGPSimulator(RecordedController(make, record, frames=record_frames, signals=signals), "vq1.r1", client=client, timeout=timeout,
                                  race_status_path=trace.with_suffix(".native.jsonl"), monitor_path=trace.with_suffix(".png"))
        try:
            result = simulator.rollout(simulator_args=simulator_args)
        finally:
            record(event="result", connection_closed=not client.connected,
                   race=None if simulator.status is None else asdict(simulator.status),
                   executable_sha256=sha256(BASE / ".runtime/vq1" / SHIPPING))

    if result is None or not result.started or not result.valid or not result.completed or result.active_gate_index != 6:
        raise AssertionError("native R1 did not finish all six gates")
    if client.connected or client.gates is None or len(client.gates) != 6:
        raise AssertionError("native R1 did not expose six gates and close its connection")
    frame = client._camera.latest
    if camera and (frame is None or frame.bgr.shape != (360, 640, 3) or frame.bgr.dtype.name != "uint8"
                   or frame.bgr.flags.writeable):
        raise AssertionError("native R1 did not expose a 640 by 360 BGR camera image")
    if camera and not cv2.imwrite(str(trace.with_suffix(".frame.png")), frame.bgr):
        raise OSError("could not save the final native camera image")
    updates = replay(make, trace, signals=signals)
    with trace.open() as source:
        rows = [json.loads(line) for line in source]
    if [row["armed"] for row in rows if row["event"] == "arm_request"] != [True, False]:
        raise AssertionError("native R1 did not arm and disarm once")
    if any(row["event"] == "telemetry" and row["message"]["mavpackettype"] == "COLLISION" for row in rows):
        raise AssertionError("native R1 reported a collision")
    sent = [row["command"] for row in rows if row["event"] == "sent"]
    if name == "r1_motors" and (len(sent) < 2 or sent[-1] != command_data(BodyRates())
            or any(command["kind"] != "MotorControls" for command in sent[:-1])):
        raise AssertionError("R1 must use only motor commands before the zero-thrust cleanup")
    print(json.dumps(dict(controller=name, gates=6, replayed_updates=updates,
                          frame_shape=None if frame is None else frame.bgr.shape, recorded_frames=record_frames, trace=str(trace))), flush=True)


def stop(signum, frame):
    raise TimeoutError("native regression exceeded 210 wall-clock seconds or was interrupted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controller", choices=CONTROLLERS)
    parser.add_argument("trace", type=Path, nargs="?")
    parser.add_argument("--camera", action="store_true", help="check native camera delivery and retain frame metadata")
    parser.add_argument("--record-frames", action="store_true", help="enable the camera and store all image pixels for replay")
    parser.add_argument("--reference", type=Path, help="sample saved motion through r1_motors (NPZ states, acceleration_rates, dt)")
    parser.add_argument("--offscreen", action="store_true", help="render natively without a movable desktop window")
    args = parser.parse_args()
    trace = args.trace or BASE / ".runtime/regressions" / f"{args.controller}-{time.time_ns()}.jsonl"
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGALRM):
        signal.signal(signum, stop)
    signal.alarm(210)
    try:
        run(args.controller, trace, args.camera, args.record_frames, reference=args.reference, offscreen=args.offscreen)
    finally:
        signal.alarm(0)


if __name__ == "__main__":
    main()
