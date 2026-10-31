"""Reset the owned native race, then transfer its fresh observation cursor."""

from dataclasses import asdict
from contextlib import closing
import argparse
import json
import math
from pathlib import Path
import signal
import shutil
import time

from miniflight.diagnostics import write_snapshot
from miniflight.position import PositionConfig
from miniflight.replay import Calls
from target.aigp.native import BASE, RaceStateReader, launch, sha256
from target.aigp.experiments.recording import ProcessClient, RaceReader, RecordedClient, packet_data, read_course, recording


def fresh_race(reader, record, timeout=120, course_packets=(), client=None, clock=time):
    client = RecordedClient(record, camera_port=None) if client is None else client
    requested, accepted, fresh, before, course = None, False, None, None, []
    reset_requests = 0
    deadline, heartbeat_at = clock.monotonic() + timeout, 0
    client.open()
    try:
        while clock.monotonic() < deadline:
            packets, states = client.poll(.02), reader.poll()
            course.extend(packet for packet in packets if packet.data.get_type() == "DATA_TRANSMISSION_HANDSHAKE"
                          or packet.data.get_type() == "ENCAPSULATED_DATA" and packet.data.data[0] == 2)
            now = clock.monotonic()
            if client.connected and now >= heartbeat_at:
                client.heartbeat()
                heartbeat_at = now + .5
            for packet in packets:
                message = packet.data
                if requested is not None and message.get_type() == "COMMAND_ACK" and message.command == 31000:
                    if message.result != 0:
                        raise RuntimeError("native reset request was rejected")
                    accepted = True
            for state in states:
                record(event="race", native_race=asdict(state))
                fresh = None
                if requested is not None and not state.started and state.valid and not state.completed and state.active_gate_index == 0 and state.time_seconds <= 0:
                    fresh = asdict(state)
            if client.connected and states and (requested is None or not accepted and now-requested >= 1):
                if requested is None:
                    before = asdict(states[-1])  # Drain old observations before requesting reset.
                client.reset()
                reset_requests += 1
                accepted, fresh = False, None
                requested = clock.monotonic()
                continue
            pose = client.telemetry.get("LOCAL_POSITION_NED")
            attitude, race = client.telemetry.get("ATTITUDE"), client.race_telemetry
            if (accepted and fresh is not None and pose is not None and attitude is not None and race is not None
                    and min(pose._host_received_at, attitude._host_received_at, race.received_at) > requested
                    and max(pose.time_boot_ms, attitude.time_boot_ms) <= race.sim_boot_time_ms
                    and (course_packets or client.gates is not None)):
                position, velocity = (pose.x, pose.y, pose.z), (pose.vx, pose.vy, pose.vz)
                if math.hypot(*position) < .1 and math.hypot(*velocity) < .1:
                    course = tuple(course) if client.gates is not None else course_packets
                    return reader, dict(before=before, after=fresh, requests=reset_requests,
                                        position_ned_m=position, velocity_ned_m_s=velocity), course
        raise TimeoutError(f"native reset stalled: connected={client.connected}, requested={requested is not None}, "
                           f"accepted={accepted}, fresh={fresh is not None}")
    finally:
        client.disconnect()


def main():
    from target.aigp.experiments.flight import run, stop
    from target.aigp.experiments.process_replay import replay
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", type=Path, help="PositionConfig JSON for this experiment")
    parser.add_argument("--controller", choices=("r1_beautiful", "r1_motors"), default="r1_motors")
    parser.add_argument("--ki", type=float, default=0.0, help="trajectory position integral gain, 1/s³")
    parser.add_argument("--count", type=int, default=0, help="number of trials; zero repeats until interrupted")
    parser.add_argument("--finish-before", type=float, required=True, help="require a native finish strictly before this race time in seconds")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--render", action="store_true", help="show the simulator window")
    parser.add_argument("--camera", action="store_true", help="record camera packets when the experiment needs images")
    parser.add_argument("--attach", type=Path, help="use this existing race-status file; keep the simulator running")
    parser.add_argument("--native-pid", type=int, help="record the process ID of the attached simulator")
    parser.add_argument("--course-recording", type=Path, help="reuse an immutable native course from this recording")
    args = parser.parse_args()
    if args.count < 0:
        parser.error("count must be nonnegative")
    if not math.isfinite(args.finish_before) or args.finish_before <= 0:
        parser.error("finish deadline must be positive and finite")
    if (args.attach is None) != (args.native_pid is None) or args.native_pid is not None and args.native_pid <= 0:
        parser.error("--attach requires a positive --native-pid")
    output = args.output or BASE / ".runtime/experiments" / f"repeat-{time.time_ns()}"
    output.mkdir(parents=True, exist_ok=False)
    native_path, ledger_path = args.attach or output / "native.jsonl", output / "trials.jsonl"
    reference_hash = sha256(args.reference)
    position_config = None if args.config is None else PositionConfig(**json.loads(args.config.read_text()))
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    signal.signal(signal.SIGALRM, stop)
    index, failures, successful = 0, 0, []
    course_packets = () if args.course_recording is None else read_course(args.course_recording)
    simulator_args = () if args.render else ("-RenderOffScreen",)
    print(f"Pitwall source: {output / 'current.json'}", flush=True)
    with ledger_path.open("x", buffering=1) as ledger, launch("vq1.r1", simulator_args, attach=args.attach is not None, race_status_path=native_path) as process, closing(RaceStateReader(native_path, .3)) as reader:
        while args.count == 0 or index < args.count:
            if shutil.disk_usage(output).free < 5 * 1024**3:
                raise OSError("Less than 5 GiB free; stopping the recording loop")
            index += 1
            trace = output / f"trial-{index:05d}.jsonl"
            row = dict(trial=index, trace=str(trace.resolve()), native_pid=args.native_pid if process is None else process.pid, native_path=str(native_path.resolve()),
                       reference_sha256=reference_hash, harness_sha256=sha256(Path(__file__)), finish_before_s=args.finish_before, started_at=time.monotonic())
            write_snapshot(output / "current.json", dict(trace=row["trace"], ledger=str(ledger_path.resolve())))
            interrupted = None
            signal.alarm(210)
            try:
                if sha256(args.reference) != reference_hash:
                    raise ValueError("reference changed during the experiment")
                with recording(trace.with_suffix(".reset.jsonl"), dict(row, process="reset", course=[packet_data(packet) for packet in course_packets])) as record:
                    calls, reset = Calls(record), None
                    client = ProcessClient(calls, camera_port=None)
                    try:
                        _, reset, course_packets = fresh_race(RaceReader(calls, reader), record, course_packets=course_packets, client=client, clock=calls)
                    finally:
                        record(event="result", reset=reset, course=[packet_data(packet) for packet in course_packets], connection_closed=not client.connected)
                row["reset"] = reset
                result = run(args.controller, trace, camera=True, record_camera=args.camera, reference=args.reference, offscreen=not args.render,
                             attach=True, race_reader=reader, course_packets=course_packets, position_config=position_config, integral_gain=args.ki, finish_before=args.finish_before)
                row.update(passed=True, finish_time_s=result.finish_time_seconds, gates=result.active_gate_index)
            except BaseException as error:
                failures += 1
                row.update(passed=False, error=dict(type=type(error).__name__, message=str(error)))
                if not isinstance(error, Exception):
                    interrupted = error
            finally:
                signal.alarm(0)
            try:
                row["replay"] = [replay(path) for path in (trace.with_suffix(".reset.jsonl"), trace) if path.exists()]
            except BaseException as error:
                row.update(passed=False, replay_error=dict(type=type(error).__name__, message=str(error)))
                interrupted = interrupted or error
            row["finished_at"] = time.monotonic()
            ledger.write(json.dumps(row) + "\n")
            print(json.dumps(row), flush=True)
            if row["passed"]:
                successful.append(trace)
                if len(successful) > 3:
                    old = successful.pop(0)
                    for artifact in old.parent.glob(old.stem + ".*"):
                        artifact.unlink()
            if interrupted is not None:
                raise interrupted
            if not row["passed"] and args.stop_on_failure:
                break
    return int(failures > 0)


if __name__ == "__main__":
    raise SystemExit(main())
