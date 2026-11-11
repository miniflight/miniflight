"""Camera/IMU gate flights, replay and post-flight scoring."""

import argparse
import base64
from contextlib import closing
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import time

from pymavlink.dialects.v20 import common as mavlink

from miniflight.replay import Calls
from target.aigp import native
from target.aigp.client import HeartbeatCadence
from target.aigp.experiments.recording import Launcher, ProcessClient, RaceReader, RecordedController, recording
from target.aigp.simulator import AIGPSimulator

from .controller import ALLOWED, CameraClock, Controller, SensorClient, SETTINGS
from .probes import Tap, build, EXPECTED_BINARY_SHA256


HERE = Path(__file__).resolve().parent


class ResetClient(ProcessClient):
    def _receive(self, message, now):
        if message.get_type() not in ALLOWED:
            return None
        return super()._receive(message, now)

    def poll(self, timeout=0.):
        return tuple(packet for packet in super().poll(timeout) if packet.data.get_type() in ALLOWED)


def reset(reader, path):
    with recording(path, dict(process="sensor_reset")) as record:
        calls = Calls(record)
        client = ResetClient(calls, camera_port=None)
        heartbeat = HeartbeatCadence()
        client.open()
        requested = None
        acknowledged = False
        deadline = time.monotonic()+120
        try:
            while time.monotonic() < deadline:
                packets = client.poll(.02)
                states = reader.poll()
                heartbeat.send_if_due(client, time.monotonic())
                if requested is None and client.connected and states:
                    client.reset()
                    requested = time.monotonic()
                for packet in packets:
                    message = packet.data
                    if message.get_type() == "COMMAND_ACK" and message.command == 31000:
                        if message.result != 0:
                            raise RuntimeError("native reset rejected")
                        acknowledged = True
                if requested is not None and acknowledged and any(not s.started and s.received_at > requested for s in states):
                    record(event="reset_ready", acknowledged=True, position_used=False)
                    return
            raise TimeoutError("sensor-only reset setup")
        finally:
            client.disconnect()


def signals(controller):
    return controller.gate.signal


def execute(calls, record, path, reader=None, producer=None, camera=None, client=None):
    if client is None:
        client = SensorClient(calls, producer_read=producer, camera_clock=camera, record_camera=True)
    controller = RecordedController(Controller, record, signals=signals)
    simulator = AIGPSimulator(controller, client=client, race_reader=RaceReader(calls, reader),
        clock=calls, timeout=.3, startup_timeout=30., race_status_path=path,
        monitor_path=Path(path).with_suffix(".unused.png"), launcher=Launcher(calls))
    error = None
    try:
        simulator.rollout(attach=True, simulator_args=("-RenderOffScreen",))
    except Exception as exception:
        error = dict(type=type(exception).__name__, message=str(exception))
        record(event="flight_error", error=error)
    instance = controller.controller
    outcome = dict(error=error, connection_closed=not client.connected, filtered_packets=client.filtered,
        phase=None if instance is None else instance.gate.phase.name,
        final_signal=None if instance is None else instance.gate.signal,
        race=None if simulator.status is None else asdict(simulator.status))
    record(event="result", **outcome)
    return outcome, client


def replay(path, poison=False):
    metadata = json.loads(path.open().readline())["metadata"]
    SETTINGS["acquisition_close_gaps"] = metadata["settings"].get("acquisition_close_gaps", False)
    SETTINGS["acquisition_largest_rim"] = metadata["settings"].get("acquisition_largest_rim", False)
    SETTINGS["lookahead_enabled"] = metadata["settings"].get("lookahead_enabled", False)
    SETTINGS["lookahead_during_final"] = metadata["settings"].get("lookahead_during_final", False)
    SETTINGS["height_alignment"] = metadata["settings"].get("height_alignment", False)
    SETTINGS.update(metadata["settings"])
    updates = [row for row in map(json.loads, path.open()) if row["event"] == "update"]
    outcomes = [row for row in map(json.loads, path.open()) if row["event"] == "result"]
    decoder = mavlink.MAVLink(None)
    encoder = mavlink.MAVLink(None)
    changed = 0
    def rows():
        nonlocal changed
        outgoing = []
        for row in map(json.loads, path.open()):
            if row["event"] == "outgoing":
                outgoing.append(row["wire"])
            elif row["event"] == "call":
                if poison and row["name"] == "poll":
                    for packet in row.get("result", []):
                        if packet["camera"]:
                            continue
                        message, = decoder.parse_buffer(base64.b64decode(packet["wire"]))
                        kind = message.get_type()
                        if kind == "LOCAL_POSITION_NED":
                            message.x, message.y, message.z = 12345., -9876., 4567.
                            message.vx, message.vy, message.vz = 200., -300., 400.
                        elif kind == "ATTITUDE":
                            message.roll, message.pitch, message.yaw = 2., -1.5, .75
                        else:
                            continue
                        packet["wire"] = base64.b64encode(message.pack(encoder)).decode("ascii")
                        changed += 1
                if poison and row["name"] == "race":
                    for race in row.get("result", []):
                        race["gate_centers"] = [[9999., -9999., 9999.]]*len(race.get("gate_centers", []))
                        race["active_gate_index"] = 0
                if row["name"] == "send" and "error" not in row:
                    if len(outgoing) != 1:
                        raise AssertionError("send does not own one outgoing packet")
                    client.expect_wire_command(outgoing[0])
                outgoing.clear()
                yield row
    recorded = iter(rows())
    calls = Calls(rows=recorded)
    client = SensorClient(calls, record_camera=True)
    def compare_updates(**row):
        if row.get("event") == "update":
            expected = updates.pop(0)
            actual = json.loads(json.dumps(row))
            if "signals" in expected:
                actual["signals"] = {key: actual["signals"][key] for key in expected["signals"]}
            omitted = {"host_time", "native_race"} if poison else {"host_time"}
            assert {k:v for k,v in expected.items() if k not in omitted} == {k:v for k,v in actual.items() if k not in omitted}, "controller update differs"
    outcome, client = execute(calls, compare_updates, path.with_name("native.jsonl"), client=client)
    calls.finish()
    assert not updates
    expected = {key:outcomes[-1][key] for key in outcome}
    compared = json.loads(json.dumps(outcome))
    if expected["final_signal"] is not None:
        compared["final_signal"] = {key: compared["final_signal"][key] for key in expected["final_signal"]}
    if poison:
        compared["race"]["gate_centers"] = expected["race"]["gate_centers"]
        compared["race"]["active_gate_index"] = expected["race"]["active_gate_index"]
    assert compared == expected, (compared, expected)
    return dict(calls=calls.count, wire_commands=client.wire_commands, exact_updates=True,
                corrupted_pose_packets=changed, native_gate_counter_zeroed=poison, outcome=outcome)


def live(root, shift):
    if not math.isfinite(shift):
        raise ValueError("The sideways setup shift must be finite")
    binaries = build()
    root.mkdir(parents=True)
    SETTINGS["shift_right_m"] = shift
    for name, filename in (("OBJECT", "object.txt"), ("DATA", "events.bin")):
        os.environ["MINIFLIGHT_CAPTURE_"+name] = "Z:"+str(root/filename)
    os.environ["MINIFLIGHT_IMU_PATH"] = "Z:"+str(root/"imu.bin")
    sources = tuple(sorted(HERE.glob("*.py")))+tuple(sorted((HERE/"native").iterdir()))
    for source in sources:
        destination = root/"source"/source.relative_to(HERE)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())
    metadata = dict(process="camera_imu", revision=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        settings=SETTINGS, shift_right_m=shift, setup="IMU controlled shift then local-origin reset", position_used_for="offline scoring",
        sources={str(source): hashlib.sha256(source.read_bytes()).hexdigest() for source in sources},
        probes=json.loads((binaries/"build.json").read_text()))
    (root/"protocol.json").write_text(json.dumps(metadata, indent=2)+"\n")
    prepare = native.prepare
    def instrument(*args, **kwargs):
        sim = prepare(*args, **kwargs)
        if native.sha256(sim/native.SHIPPING) != EXPECTED_BINARY_SHA256:
            raise ValueError("Camera/IMU probes require the pinned VQ1 executable")
        path = sim/native.BINARIES/"Mods/DirectVQ1/Scripts/main.lua"
        with path.open("a") as stream:
            stream.write("\n"+(HERE/"native/identify.lua").read_text())
        return sim
    native.prepare = instrument
    environment = dict(os.environ, WINEPREFIX=str(native.BASE/".runtime/vq1-wine"), WINEDEBUG="-all")
    try:
        with native.launch("vq1.r1", ("-RenderOffScreen",), race_status_path=root/"native.jsonl") as process, closing(native.RaceStateReader(root/"native.jsonl", .3)) as reader:
            deadline = time.monotonic()+120
            while not (root/"object.txt").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise TimeoutError("scene camera identification")
                time.sleep(.1)
            for dll in (binaries/"camera.dll", binaries/"imu.dll"):
                subprocess.run([native.wine_commands()[0], str(binaries/"load.exe"), "Z:"+str(dll)], env=environment, timeout=40, check=True)
            with closing(Tap(root/"imu.bin")) as tap, closing(CameraClock(root/"events.bin")) as camera:
                tap.wait_for_producer(process)
                reset(reader, root/"reset.jsonl")
                time.sleep(.75)
                bounds = tap.calibrate()
                (root/"clock.json").write_text(json.dumps(dict(bounds=bounds, frequency=tap.frequency), indent=2)+"\n")
                tap.discard()
                def producer():
                    batch = tap.read()
                    if batch["rows"]:
                        batch["published_ns"] = batch["rows"][-1][-1]*10**9//tap.frequency+bounds[0]
                    return batch
                with recording(root/"flight.jsonl", metadata) as record:
                    count = 0
                    def bounded_record(**row):
                        nonlocal count
                        record(**row)
                        count += 1
                        if count % 64 == 0 and (root/"flight.jsonl").stat().st_size > 128*1024**2:
                            raise RuntimeError("temporary flight trace exceeded 128 MiB")
                    outcome, _ = execute(Calls(bounded_record), bounded_record, root/"native.jsonl", reader, producer, camera.read)
                    print(json.dumps(outcome), flush=True)
    finally:
        native.prepare = prepare
        prepare("vq1")
    result = replay(root/"flight.jsonl")
    (root/"replay.json").write_text(json.dumps(result, indent=2)+"\n")
    print("replay", json.dumps(result), flush=True)
    from .score import score
    return score(root)


def one(root, shift):
    """One corrected gate; remove pixels and raw traces after scoring and replay."""
    root.mkdir(parents=True)
    original = dict(SETTINGS)
    SETTINGS.update(gates_to_pass=1, camera_correction=True, camera_window_s=.5, speed_m_s=2.,
                    beyond_m=1.5, maximum_blind_s=4.5, flight_timeout_s=20.,
                    lookahead_enabled=False, lookahead_during_final=False, height_alignment=False)
    try:
        with tempfile.TemporaryDirectory(prefix="raw-", dir=root) as temporary:
            raw = Path(temporary)/"flight"
            try:
                result = live(raw, shift)
                checked = replay(raw/"flight.jsonl", poison=True)
                checked.pop("outcome")
                (root/"input-independence.json").write_text(json.dumps(checked, indent=2)+"\n")
                return result
            except Exception as error:
                (root/"failure.json").write_text(json.dumps(dict(type=type(error).__name__, message=str(error)), indent=2)+"\n")
                raise
            finally:
                for name in ("protocol.json", "clock.json", "score.json", "replay.json", "camera-errors.csv"):
                    if (raw/name).exists():
                        shutil.copyfile(raw/name, root/name)
                size = sum(path.stat().st_size for path in raw.rglob("*") if path.is_file())
                (root/"retention.json").write_text(json.dumps(dict(temporary_bytes_removed=size, raw_recordings_retained=False))+"\n")
    finally:
        SETTINGS.clear()
        SETTINGS.update(original)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("build", "one", "live", "replay", "score", "video"))
    parser.add_argument("path", type=Path, nargs="?")
    parser.add_argument("--shift", type=float, default=-.5)
    parser.add_argument("--poison", action="store_true")
    args = parser.parse_args()
    if args.mode == "build":
        print(build())
    elif args.path is None:
        parser.error("live, replay, score and video require a path")
    elif args.mode in ("live", "one"):
        run = one if args.mode == "one" else live
        result = run(args.path.resolve(), args.shift)
        used = args.mode != "one" or all(result.get("estimation", {}).get(name, 0) > 0
            for name in ("camera_position_updates", "camera_velocity_updates"))
        if not result["passed_without_reset"] or not used:
            raise SystemExit(1)
    elif args.mode == "replay":
        print(json.dumps(replay(args.path.resolve(), args.poison)), flush=True)
    elif args.mode == "score":
        from .score import score
        if not score(args.path.resolve())["passed_without_reset"]:
            raise SystemExit(1)
    else:
        from .video import render
        render(args.path.resolve())
