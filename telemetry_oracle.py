#!/usr/bin/env python3
"""Read-only MSP telemetry oracle with console output and CSV logging."""

from __future__ import annotations

import argparse
import csv
import json
import os
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from miniflight.msp import (
    MSP_ANALOG,
    MSP_ATTITUDE,
    MSP_ALTITUDE,
    MSP_RAW_IMU,
    MSP_STATUS,
    MSPParser,
    MSPSerial,
    open_serial,
    serial_devices,
)
from miniflight.probe import MspMachineProbe, machine_profile_from_responses
from miniflight.record import FlightRecord


BAUD = int(os.environ.get("MINIFLIGHT_SERIAL_BAUD", "115200"))
DEVICE_OVERRIDE = os.environ.get("MINIFLIGHT_SERIAL")

SLOW_POLL_SECONDS = {
    MSP_STATUS: 1.0,
    MSP_ANALOG: 1.0,
    MSP_ALTITUDE: 0.5,
}


def decode_attitude(payload: bytes) -> Optional[tuple[float, float, float]]:
    if len(payload) < 6:
        return None
    roll, pitch, yaw = struct.unpack_from("<hhh", payload)
    return roll / 10.0, pitch / 10.0, float(yaw)


def decode_raw_imu(payload: bytes) -> Optional[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    if len(payload) < 18:
        return None
    values = struct.unpack_from("<9h", payload)
    return values[0:3], values[3:6]


def safe_float(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.4f}"


def format_status_line(sample: dict[str, object]) -> str:
    return (
        f"#{sample['index']:04d} "
        f"{sample['link']:<7} "
        f"age={sample['age_ms']:>6}ms "
        f"loop={safe_float(sample['loop_hz'])}Hz "
        f"rpy=({safe_float(sample['roll'])},{safe_float(sample['pitch'])},{safe_float(sample['yaw'])}) "
        f"acc=({safe_float(sample['acc_x'])},{safe_float(sample['acc_y'])},{safe_float(sample['acc_z'])}) "
        f"gyro=({safe_float(sample['gyr_x'])},{safe_float(sample['gyr_y'])},{safe_float(sample['gyr_z'])}) "
        f"mean_acc={safe_float(sample['mean_acc'])} "
        f"mean_gyro={safe_float(sample['mean_gyr'])}"
    )


def run_oracle(
    device: str,
    out_path: Path,
    max_samples: Optional[int] = None,
    record: Optional[FlightRecord] = None,
) -> None:
    if record is not None:
        record.link("opening", device)
    fd: Optional[int] = None
    try:
        fd = open_serial(device, BAUD)
        client = MSPSerial(
            fd,
            on_frame=record.response if record is not None else None,
            on_request=record.request if record is not None else None,
        )
        profile = MspMachineProbe().inspect(client)
        if record is not None:
            record.machine(
                device,
                profile,
                {
                    "attitude": "alternate",
                    "raw_imu": "alternate",
                    "status_s": SLOW_POLL_SECONDS[MSP_STATUS],
                    "analog_s": SLOW_POLL_SECONDS[MSP_ANALOG],
                    "altitude_s": SLOW_POLL_SECONDS[MSP_ALTITUDE],
                },
            )
            record.link("live", device)
        print(
            f"Connecting to {device} | variant={profile.controller} "
            f"board={profile.board} fw={profile.firmware} api={profile.api_version}"
        )

        link_state = "LIVE"
        fast_tick = 0
        now = time.monotonic()
        next_slow_poll = {command: now + period for command, period in SLOW_POLL_SECONDS.items()}
        last_response = time.monotonic()
        status = {
            "armed": None,
            "loop_hz": None,
            "telemetry_hz": None,
            "battery_v": None,
            "mah": None,
            "rssi_percent": None,
            "altitude_m": None,
            "vario_mps": None,
        }
        signals = {
            "roll": None,
            "pitch": None,
            "yaw": None,
            "accel": None,
            "gyro": None,
        }
        acc_mean = [0.0, 0.0, 0.0]
        gyr_mean = [0.0, 0.0, 0.0]
        mean_count = 0

        rate_window_start = last_response
        frames_total = 0
        frames_window = 0
        imu_window = 0
        attitude_window = 0
        rates = {"frames_per_second": 0.0, "imu_per_second": 0.0, "attitude_per_second": 0.0}

        with out_path.open("w", newline="", encoding="utf-8") as out_file:
            writer = csv.writer(out_file)
            writer.writerow(
                [
                    "ts_utc",
                    "index",
                    "device",
                    "link_state",
                    "age_ms",
                    "loop_hz",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                    "accel_x",
                    "accel_y",
                    "accel_z",
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "mean_accel_mag",
                    "mean_gyro_mag",
                    "armed",
                    "battery_v",
                    "mah",
                    "rssi_percent",
                    "altitude_m",
                    "vario_mps",
                    "frames_per_second",
                    "imu_per_second",
                    "attitude_per_second",
                ]
            )

            sample_idx = 0
            while True:
                loop_started = time.monotonic()
                loop_now = loop_started

                if link_state != "LIVE" and loop_now - last_response < 1.0:
                    link_state = "CONNECTING"

                if fast_tick % 2 == 0:
                    command = MSP_ATTITUDE
                else:
                    due = [cmd for cmd, due_time in next_slow_poll.items() if loop_now >= due_time]
                    if due:
                        command = min(due, key=next_slow_poll.get)
                        next_slow_poll[command] = loop_now + SLOW_POLL_SECONDS[command]
                    else:
                        command = MSP_RAW_IMU
                fast_tick += 1

                payload = client.request(command, timeout=0.2)
                received = time.monotonic()
                age_ms = round(max(0.0, received - last_response) * 1000, 1)
                frames_window += 1
                frames_total += 1
                if payload is not None:
                    status["telemetry_hz"] = (
                        round(1.0 / (received - last_response), 2)
                        if received > last_response
                        else None
                    )
                    last_response = received
                    link_state = "LIVE"

                    if command == MSP_ATTITUDE:
                        attitude = decode_attitude(payload)
                        if attitude is not None:
                            signals["roll"], signals["pitch"], signals["yaw"] = attitude
                            attitude_window += 1
                    elif command == MSP_RAW_IMU:
                        imu = decode_raw_imu(payload)
                        if imu is not None:
                            accel, gyro = imu
                            signals["accel"] = accel
                            signals["gyro"] = gyro
                            mean_count += 1
                            for index in range(3):
                                acc_mean[index] += (accel[index] - acc_mean[index]) / mean_count
                                gyr_mean[index] += (gyro[index] - gyr_mean[index]) / mean_count
                            imu_window += 1
                    elif command == MSP_STATUS and len(payload) >= 13:
                        cycle_time_us = struct.unpack_from("<H", payload, 0)[0]
                        status["loop_hz"] = round(1_000_000 / cycle_time_us, 2) if cycle_time_us else None
                    elif command == MSP_ANALOG and len(payload) >= 7:
                        voltage, mah, rssi, current = struct.unpack_from("<BHHH", payload)
                        status["battery_v"] = voltage / 10.0
                        status["mah"] = mah
                        status["rssi_percent"] = round(min(100.0, rssi * 100.0 / 1023.0), 1)
                        status["current_a"] = current / 100.0
                    elif command == MSP_ALTITUDE and len(payload) >= 6:
                        altitude_cm, vario_cms = struct.unpack_from("<ih", payload)
                        status["altitude_m"] = altitude_cm / 100.0
                        status["vario_mps"] = vario_cms / 100.0

                    if status["loop_hz"] is None:
                        status["loop_hz"] = None

                    elapsed = loop_now - rate_window_start
                    if elapsed >= 1.0:
                        rates["frames_per_second"] = round(frames_window / elapsed, 1)
                        rates["imu_per_second"] = round(imu_window / elapsed, 1)
                        rates["attitude_per_second"] = round(attitude_window / elapsed, 1)
                        rate_window_start = loop_now
                        frames_window = 0
                        imu_window = 0
                        attitude_window = 0

                    roll = signals["roll"]
                    pitch = signals["pitch"]
                    yaw = signals["yaw"]
                    acc = signals["accel"]
                    gyr = signals["gyro"]
                    mean_acc_mag = sum(value * value for value in acc_mean) ** 0.5 if mean_count else None
                    mean_gyr_mag = sum(value * value for value in gyr_mean) ** 0.5 if mean_count else None
                    sample = {
                        "index": sample_idx,
                        "link": link_state,
                        "age_ms": age_ms,
                        "loop_hz": status["loop_hz"] if status["loop_hz"] is not None else status["telemetry_hz"],
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw,
                        "acc_x": acc[0] if acc else None,
                        "acc_y": acc[1] if acc else None,
                        "acc_z": acc[2] if acc else None,
                        "gyr_x": gyr[0] if gyr else None,
                        "gyr_y": gyr[1] if gyr else None,
                        "gyr_z": gyr[2] if gyr else None,
                        "mean_acc": mean_acc_mag,
                        "mean_gyr": mean_gyr_mag,
                    }

                    print(format_status_line(sample), flush=True)
                    writer.writerow(
                        [
                            datetime.now(tz=timezone.utc).isoformat(),
                            sample_idx,
                            device,
                            link_state,
                            age_ms,
                            safe_float(sample["loop_hz"]),
                            "" if roll is None else f"{roll:.3f}",
                            "" if pitch is None else f"{pitch:.3f}",
                            "" if yaw is None else f"{yaw:.3f}",
                            "" if acc is None else acc[0],
                            "" if acc is None else acc[1],
                            "" if acc is None else acc[2],
                            "" if gyr is None else gyr[0],
                            "" if gyr is None else gyr[1],
                            "" if gyr is None else gyr[2],
                            "" if mean_acc_mag is None else f"{mean_acc_mag:.6f}",
                            "" if mean_gyr_mag is None else f"{mean_gyr_mag:.6f}",
                            "" if status["armed"] is None else status["armed"],
                            status["battery_v"] or "",
                            status["mah"] or "",
                            status["rssi_percent"] or "",
                            status["altitude_m"] or "",
                            status["vario_mps"] or "",
                            rates["frames_per_second"],
                            rates["imu_per_second"],
                            rates["attitude_per_second"],
                        ]
                    )
                    sample_idx += 1
                    if max_samples is not None and sample_idx >= max_samples:
                        return

                elif time.monotonic() - last_response > 1.5:
                    link_state = "OFFLINE"
                    status_str = f"{time.time():.3f} connection lost: no MSP response"
                    print(status_str)
                    raise RuntimeError("MSP telemetry stalled")

                time.sleep(max(0.0, 0.005 - (time.monotonic() - loop_started)))
    finally:
        if record is not None:
            record.link("closed", device)
        if fd is not None:
            os.close(fd)


def replay_capture(path: Path) -> int:
    """Decode a raw MSP capture without a serial device."""
    parser = MSPParser()
    frame_count = 0
    attitude_count = 0
    imu_count = 0

    with path.open(encoding="utf-8") as capture_file:
        try:
            header = json.loads(capture_file.readline())
        except json.JSONDecodeError as error:
            print(f"Invalid capture header: {error}", file=sys.stderr)
            return 2
        capture_format = header.get("format")
        if capture_format not in {"miniflight-msp-capture", "miniflight-flight-record"}:
            print("Not a Miniflight MSP capture.", file=sys.stderr)
            return 2

        for line_number, line in enumerate(capture_file, start=2):
            try:
                record = json.loads(line)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                print(f"Invalid capture frame at line {line_number}: {error}", file=sys.stderr)
                return 2
            if capture_format == "miniflight-flight-record":
                if record.get("type") != "msp_response":
                    continue
            try:
                parser.feed(bytes.fromhex(record["frame_hex"]))
            except (KeyError, TypeError, ValueError) as error:
                print(f"Invalid capture frame at line {line_number}: {error}", file=sys.stderr)
                return 2

            while (frame := parser.pop()) is not None:
                _, command, payload = frame
                frame_count += 1
                if command == MSP_ATTITUDE:
                    attitude = decode_attitude(payload)
                    if attitude is not None:
                        attitude_count += 1
                        roll, pitch, yaw = attitude
                        print(
                            f"ATTITUDE {attitude_count:04d} "
                            f"rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})"
                        )
                elif command == MSP_RAW_IMU:
                    imu = decode_raw_imu(payload)
                    if imu is not None:
                        imu_count += 1
                        accel, gyro = imu
                        print(f"IMU {imu_count:04d} accel={accel} gyro={gyro}")

    print(
        f"Replay complete frames={frame_count} "
        f"attitude={attitude_count} imu={imu_count}"
    )
    return 0


def report_capture(path: Path) -> int:
    """Print machine facts reconstructed from one raw capture."""
    parser = MSPParser()
    responses: dict[int, bytes] = {}
    frame_count = 0
    manifest = None

    with path.open(encoding="utf-8") as capture_file:
        try:
            header = json.loads(capture_file.readline())
        except json.JSONDecodeError as error:
            print(f"Invalid capture header: {error}", file=sys.stderr)
            return 2
        capture_format = header.get("format")
        if capture_format not in {"miniflight-msp-capture", "miniflight-flight-record"}:
            print("Not a Miniflight MSP capture.", file=sys.stderr)
            return 2

        for line_number, line in enumerate(capture_file, start=2):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                print(f"Invalid capture frame at line {line_number}: {error}", file=sys.stderr)
                return 2
            if capture_format == "miniflight-flight-record":
                if record.get("type") == "machine":
                    manifest = record
                if record.get("type") != "msp_response":
                    continue
            try:
                parser.feed(bytes.fromhex(record["frame_hex"]))
            except (KeyError, TypeError, ValueError) as error:
                print(f"Invalid capture frame at line {line_number}: {error}", file=sys.stderr)
                return 2
            while (frame := parser.pop()) is not None:
                direction, command, payload = frame
                if direction == ">":
                    responses[command] = payload
                    frame_count += 1

    profile = machine_profile_from_responses(responses)
    print(f"frames {frame_count}")
    print(f"controller {profile.controller or '-'}")
    print(f"board {profile.board or '-'}")
    print(f"firmware {profile.firmware or '-'}")
    print(f"api {profile.api_version or '-'}")
    print(f"sensors {' '.join(sorted(profile.sensors)) or '-'}")
    print(f"motor_count {profile.motor_count if profile.motor_count is not None else '-'}")
    if manifest is not None:
        print(f"device {manifest['device']}")
        print(f"poll_plan {json.dumps(manifest['poll_plan'], sort_keys=True, separators=(',', ':'))}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only MSP telemetry oracle.")
    parser.add_argument("--device", default=None, help="Serial device path")
    parser.add_argument(
        "--out",
        default=None,
        help="CSV output path (defaults to oracle_<timestamp>.csv)",
    )
    parser.add_argument("--samples", type=int, default=None, help="Max samples before exit")
    parser.add_argument("--record-out", default=None, help="Append-only flight record path")
    parser.add_argument("--replay", default=None, help="Replay a raw MSP capture")
    parser.add_argument("--report", default=None, help="Report machine facts from a raw capture")
    args = parser.parse_args()

    if args.replay:
        capture_path = Path(args.replay).expanduser().resolve()
        if not capture_path.is_file():
            print(f"Capture not found: {capture_path}", file=sys.stderr)
            return 2
        return replay_capture(capture_path)

    if args.report:
        capture_path = Path(args.report).expanduser().resolve()
        if not capture_path.is_file():
            print(f"Capture not found: {capture_path}", file=sys.stderr)
            return 2
        return report_capture(capture_path)

    if args.device:
        devices = [args.device]
        if not os.path.exists(args.device):
            print(f"Device not found: {args.device}", file=sys.stderr)
            return 2
    else:
        devices = serial_devices(DEVICE_OVERRIDE)
    if not devices:
        print("No USB serial device found.")
        return 2

    if args.out is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.out = f"telemetry_oracle_{stamp}.csv"

    out_path = Path(args.out).expanduser().resolve()
    print(f"Writing CSV to {out_path}")
    record = None
    if args.record_out:
        record_path = Path(args.record_out).expanduser().resolve()
        print(f"Writing flight record to {record_path}")
        record = FlightRecord(record_path)

    try:
        for device in devices:
            try:
                run_oracle(device, out_path, max_samples=args.samples, record=record)
                return 0
            except PermissionError:
                print(f"{device} is busy or not permitted. Try closing configurator.")
            except OSError as error:
                print(f"{device} failed: {error}")
            except KeyboardInterrupt:
                print("\nStopped by user.")
                return 0
    finally:
        if record is not None:
            record.close()

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
