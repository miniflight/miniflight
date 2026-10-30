"""Record native execution and verify the R1 race when requested."""

import argparse
from contextlib import ExitStack, closing
from dataclasses import asdict
from functools import partial
import json
import inspect
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from miniflight import BodyRates
from miniflight.reference import sample_motion, sample_values
from miniflight.replay import Calls
from target.aigp.controllers.r1_beautiful import CONFIG, Controller as PIDGates
from target.aigp.controllers.r1_motors import Controller as MotorGates
from target.aigp.controllers import load_controller
from target.aigp.experiments.recording import Launcher, ProcessClient, RaceReader, RecordedController, command_data, recording, replay
from target.aigp.experiments.r1_signals import motor_signals, r1_signals
from target.aigp.native import BASE, SHIPPING, TARGETS, RaceStateReader, sha256
from target.aigp.simulator import AIGPSimulator


CONTROLLERS = ("r1_gates", "r1_body_rates", "r1_beautiful", "r1_motors")
ROOT = BASE.parents[1]


def flight_controller(name, reference=None, position_config=None, integral_gain=0.0):
    signals, reference_duration = None, None
    make = load_controller(name)
    if reference is not None:
        if name not in ("r1_beautiful", "r1_motors"):
            raise ValueError("a recorded reference requires r1_beautiful or r1_motors")
        reference = Path(reference)
        with np.load(reference) as data:
            times = data["times"] if "times" in data else None
            trajectory = partial(sample_motion, data["states"], data["acceleration_rates"], float(data["dt"]), times=times)
            rate_reference = partial(sample_values, data["body_rates"], data["body_rate_times"]) if "body_rates" in data else None
            reference_duration = float(times[-1]) if times is not None else len(data["acceleration_rates"]) * float(data["dt"])
        config = CONFIG if position_config is None else position_config
        follower = partial(PIDGates, reference=trajectory, rate_reference=rate_reference, config=config,
                           kp=config.position_gain * config.velocity_gain, kd=config.velocity_gain, ki=integral_gain)
        make = (lambda: MotorGates(position=follower())) if name == "r1_motors" else follower
        signals = r1_signals if name == "r1_beautiful" else lambda controller: {**r1_signals(controller.position), **motor_signals(controller)}
    if name == "r1_motors" and signals is None:
        signals = motor_signals
    return make, signals, reference_duration


def rollout(name, trace, camera=False, record_frames=False, signals=None, reference=None, offscreen=False, attach=False,
            race_reader=None, course_packets=(), position_config=None, integral_gain=0.0, finish_before=None, *,
            target="vq1.r1", hz=50.0, timeout=None, startup_timeout=120.0, simulator_args=None, timesync=False):
    trace = Path(trace)
    camera = camera or record_frames
    simulator_args = (("-RenderOffScreen",) if offscreen else ()) if simulator_args is None else tuple(simulator_args)
    reference = None if reference is None else Path(reference)
    make, default_signals, reference_duration = flight_controller(name, reference, position_config, integral_gain)
    signals = default_signals if signals is None else signals
    if timeout is None:
        timeout = .3 if name == "r1_motors" else 1.0
    sources = subprocess.check_output(["git", "ls-files", "*.py", "*.lua"], cwd=ROOT, text=True).splitlines()
    metadata = dict(target=target, controller=name, hz=hz, timeout=timeout, startup_timeout_s=startup_timeout, finish_before_s=finish_before, camera=camera, recorded_frames=record_frames, attach=attach,
                    simulator_args=simulator_args, timesync=timesync, process="flight", race_status_path=str(trace.with_suffix(".native.jsonl")),
                    reference=None if reference is None else dict(path=str(reference.resolve()), sha256=sha256(reference), duration_s=reference_duration),
                    revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    sources={path: sha256(ROOT / path) for path in sources})
    controller_source = Path(inspect.getfile(load_controller(name))).resolve()
    key = str(controller_source.relative_to(ROOT)) if controller_source.is_relative_to(ROOT) else str(controller_source)
    metadata["sources"][key] = sha256(controller_source)
    executable = BASE / ".runtime" / TARGETS[target][0] / SHIPPING
    if name == "r1_motors" or reference is not None:
        controller = make()
        position = controller.position if name == "r1_motors" else controller
        metadata["controller_config"] = asdict(position.config)
        metadata["trajectory_pid"] = dict(kp=position.kp, ki=position.ki, kd=position.kd)
        if name == "r1_motors":
            metadata["rate_pid"] = dict(kp=controller.kp, ki=controller.ki)
    if race_reader is not None:
        metadata["native_race_source"] = dict(path=str(race_reader.path.resolve()), initial_offset=race_reader.stream.tell())
    result = None
    with recording(trace, metadata) as record, ExitStack() as cleanup:
        calls = Calls(record)
        if race_reader is None:
            race_reader = cleanup.enter_context(closing(RaceStateReader(trace.with_suffix(".native.jsonl"), timeout)))
        client = ProcessClient(calls, camera_port=5600 if camera else None, course_packets=course_packets, timesync=timesync,
                                preview=trace.with_suffix(".frame.jpg") if camera else None)
        simulator = AIGPSimulator(RecordedController(make, record, frames=record_frames, signals=signals), target, hz=hz, client=client, timeout=timeout, startup_timeout=startup_timeout,
                                  race_status_path=trace.with_suffix(".native.jsonl"), monitor_path=trace.with_suffix(".png"), race_reader=RaceReader(calls, race_reader), clock=calls, finish_before=finish_before, launcher=Launcher(calls))
        try:
            result = simulator.rollout(attach=attach, simulator_args=simulator_args)
        finally:
            record(event="result", connection_closed=not client.connected,
                   returned=None if result is None else asdict(result),
                   race=None if simulator.status is None else asdict(simulator.status),
                   executable_sha256=sha256(executable) if executable.is_file() else None)

    simulator.save_monitor()
    return result, simulator, make, signals


def run(name, trace, camera=False, record_frames=False, **kwargs):
    result, simulator, make, signals = rollout(name, trace, camera, record_frames, **kwargs)
    return check(name, trace, result, simulator.client, make, camera or record_frames, record_frames, signals)


def check(name, trace, result, client, make, camera, record_frames, signals):
    if result is None or not result.started or not result.valid or not result.completed or result.active_gate_index != 6:
        raise AssertionError("native R1 did not finish all six gates")
    if client.connected or client.gates is None or len(client.gates) != 6:
        raise AssertionError("native R1 did not expose six gates and close its connection")
    frame = None if client.state is None else client.state.frame
    if camera and (frame is None or frame.bgr.shape != (360, 640, 3) or frame.bgr.dtype.name != "uint8"
                   or frame.bgr.flags.writeable):
        raise AssertionError("native R1 did not expose a 640 by 360 BGR camera image")
    updates = replay(make, trace, signals=signals)
    with trace.open() as source:
        rows = [row for line in source if (row := json.loads(line))["event"] in ("arm_request", "telemetry", "sent")]
    requests = [row for row in rows if row["event"] == "arm_request"]
    if [row["armed"] for row in requests] != [True, False]:
        raise AssertionError("native R1 did not request arm and disarm once")
    for index, request in enumerate(requests):
        until = requests[index+1]["host_time"] if index+1 < len(requests) else float("inf")
        if not any(row["event"] == "telemetry" and row["message"]["mavpackettype"] == "HEARTBEAT"
                   and request["host_time"] < row["host_time"] < until
                   and bool(row["message"]["base_mode"] & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == request["armed"] for row in rows):
            raise AssertionError(f"native R1 did not report armed={request['armed']} after the request")
    if any(row["event"] == "telemetry" and row["message"]["mavpackettype"] == "COLLISION" for row in rows):
        raise AssertionError("native R1 reported a collision")
    sent = [row["command"] for row in rows if row["event"] == "sent"]
    if name == "r1_motors" and (len(sent) < 2 or sent[-1] != command_data(BodyRates())
            or any(command["kind"] != "MotorControls" for command in sent[:-1])):
        raise AssertionError("R1 must use only motor commands before the zero-thrust cleanup")
    print(json.dumps(dict(controller=name, gates=6, replayed_updates=updates,
                          frame_shape=None if frame is None else frame.bgr.shape, recorded_frames=record_frames, trace=str(trace))), flush=True)
    return result


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
    parser.add_argument("--finish-before", type=float, help="require a native finish strictly before this race time in seconds")
    args = parser.parse_args()
    trace = args.trace or BASE / ".runtime/regressions" / f"{args.controller}-{time.time_ns()}.jsonl"
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGALRM):
        signal.signal(signum, stop)
    signal.alarm(210)
    try:
        run(args.controller, trace, args.camera, args.record_frames, reference=args.reference, offscreen=args.offscreen, finish_before=args.finish_before)
    finally:
        signal.alarm(0)


if __name__ == "__main__":
    main()
