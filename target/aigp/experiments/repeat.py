"""Reset the owned native race, then transfer its fresh observation cursor."""

from dataclasses import asdict
import argparse
import json
import math
from pathlib import Path
import signal
import time

from miniflight.diagnostics import write_snapshot
from target.aigp.client import SimulatorClient
from target.aigp.native import BASE, RaceStateReader, launch, sha256


def fresh_race(path, timeout=120, course_packets=()):
    reader, client = RaceStateReader(path, .3), SimulatorClient(camera_port=None)
    requested, accepted, fresh, before, course = None, False, None, None, []
    deadline, heartbeat_at = time.monotonic() + timeout, 0
    client.open()
    try:
        while time.monotonic() < deadline:
            packets, states = client.poll(.02), reader.poll()
            course.extend(packet for packet in packets if packet.data.get_type() == "DATA_TRANSMISSION_HANDSHAKE"
                          or packet.data.get_type() == "ENCAPSULATED_DATA" and packet.data.data[0] == 2)
            now = time.monotonic()
            if client.connected and now >= heartbeat_at:
                client.heartbeat()
                heartbeat_at = now + .5
            if requested is None and client.connected and states:
                before = asdict(states[-1])  # Drain old observations before requesting reset.
                client.reset()
                requested = time.monotonic()
                continue
            if requested is None:
                continue
            for packet in packets:
                message = packet.data
                if message.get_type() == "COMMAND_ACK" and message.command == 31000:
                    if message.result != 0:
                        raise RuntimeError("native reset request was rejected")
                    accepted = True
            for state in states:
                if not state.started and state.valid and not state.completed and state.active_gate_index == 0 and state.time_seconds == 0:
                    fresh = asdict(state)
            pose = client.telemetry.get("LOCAL_POSITION_NED")
            if (accepted and fresh is not None and pose is not None and pose._host_received_at > requested
                    and (course_packets or client.gates is not None)):
                position, velocity = (pose.x, pose.y, pose.z), (pose.vx, pose.vy, pose.vz)
                if math.hypot(*position) < .1 and math.hypot(*velocity) < .1:
                    course = tuple(course) if client.gates is not None else course_packets
                    return reader, dict(before=before, after=fresh, position_ned_m=position, velocity_ned_m_s=velocity), course
        raise TimeoutError("native reset did not expose a fresh stationary starting grid")
    except BaseException:
        reader.close()
        raise
    finally:
        client.disconnect()


def main():
    from test.aigp_regression import run, stop
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--count", type=int, default=0, help="number of trials; zero repeats until interrupted")
    parser.add_argument("--stop-on-failure", action="store_true")
    args = parser.parse_args()
    if args.count < 0:
        parser.error("count must be nonnegative")
    output = args.output or BASE / ".runtime/experiments" / f"repeat-{time.time_ns()}"
    output.mkdir(parents=True, exist_ok=False)
    native_path, ledger_path = output / "native.jsonl", output / "trials.jsonl"
    reference_hash = sha256(args.reference)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)
    signal.signal(signal.SIGALRM, stop)
    index, failures, course_packets = 0, 0, ()
    print(f"Pitwall source: {output / 'current.json'}", flush=True)
    with ledger_path.open("x", buffering=1) as ledger, launch("vq1.r1", ("-RenderOffScreen",), race_status_path=native_path) as process:
        while args.count == 0 or index < args.count:
            index += 1
            trace = output / f"trial-{index:05d}.jsonl"
            row = dict(trial=index, trace=str(trace.resolve()), native_pid=process.pid, native_path=str(native_path.resolve()),
                       reference_sha256=reference_hash, started_at=time.monotonic())
            write_snapshot(output / "current.json", dict(trace=row["trace"], ledger=str(ledger_path.resolve())))
            reader, interrupted = None, None
            signal.alarm(210)
            try:
                if sha256(args.reference) != reference_hash:
                    raise ValueError("reference changed during the experiment")
                reader, reset, course_packets = fresh_race(native_path, course_packets=course_packets)
                row["reset"] = reset
                result = run("r1_motors", trace, camera=True, reference=args.reference, offscreen=True,
                             attach=True, race_reader=reader, course_packets=course_packets)
                row.update(passed=True, finish_time_s=result.finish_time_seconds, gates=result.active_gate_index)
            except BaseException as error:
                failures += 1
                row.update(passed=False, error=dict(type=type(error).__name__, message=str(error)))
                if not isinstance(error, Exception):
                    interrupted = error
            finally:
                signal.alarm(0)
                if reader is not None:
                    reader.close()
            row["finished_at"] = time.monotonic()
            ledger.write(json.dumps(row) + "\n")
            print(json.dumps(row), flush=True)
            if interrupted is not None:
                raise interrupted
            if not row["passed"] and args.stop_on_failure:
                break
    return int(failures > 0)


if __name__ == "__main__":
    raise SystemExit(main())
