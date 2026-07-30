#!/usr/bin/env python3
"""Read-only MSP telemetry oracle with console output and CSV logging."""

from __future__ import annotations

import argparse
import csv
import json
import math
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
    xor_checksum,
)
from miniflight.probe import MspMachineProbe, machine_profile_from_responses
from miniflight.record import FlightRecord, read_flight_record


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
    max_seconds: Optional[float] = None,
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
                    "attitude": "unused non-IMU slots",
                    "raw_imu": "every second request",
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
        capture_started = time.monotonic()

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
                if max_seconds is not None and loop_started - capture_started >= max_seconds:
                    return
                loop_now = loop_started

                if link_state != "LIVE" and loop_now - last_response < 1.0:
                    link_state = "CONNECTING"

                if fast_tick % 2 == 1:
                    command = MSP_RAW_IMU
                else:
                    due = [cmd for cmd, due_time in next_slow_poll.items() if loop_now >= due_time]
                    if due:
                        command = min(due, key=next_slow_poll.get)
                        next_slow_poll[command] = loop_now + SLOW_POLL_SECONDS[command]
                    else:
                        command = MSP_ATTITUDE
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


def _response_frame(record: dict) -> tuple[str, int, bytes]:
    parser = MSPParser()
    try:
        parser.feed(bytes.fromhex(record["frame_hex"]))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid MSP response at sequence {record['sequence']}") from error
    frame = parser.pop()
    if frame is None:
        raise ValueError(f"invalid MSP response at sequence {record['sequence']}")
    return frame


def _request_command(record: dict) -> int:
    try:
        frame = bytes.fromhex(record["frame_hex"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid MSP request at sequence {record['sequence']}") from error
    if len(frame) != 6 or frame[:3] != b"$M<" or xor_checksum(frame[3:-1]) != frame[-1]:
        raise ValueError(f"invalid MSP request at sequence {record['sequence']}")
    return frame[4]


def replay_capture(path: Path) -> int:
    """Decode a flight record without a serial device."""
    frame_count = 0
    attitude_count = 0
    imu_count = 0
    try:
        for record in read_flight_record(path):
            if record.get("type") != "msp_response":
                continue
            _, command, payload = _response_frame(record)
            frame_count += 1
            if command == MSP_ATTITUDE and (attitude := decode_attitude(payload)) is not None:
                attitude_count += 1
                roll, pitch, yaw = attitude
                print(f"ATTITUDE {attitude_count:04d} rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})")
            elif command == MSP_RAW_IMU and (imu := decode_raw_imu(payload)) is not None:
                imu_count += 1
                accel, gyro = imu
                print(f"IMU {imu_count:04d} accel={accel} gyro={gyro}")
    except ValueError as error:
        print(f"Invalid flight record: {error}", file=sys.stderr)
        return 2

    print(
        f"Replay complete frames={frame_count} "
        f"attitude={attitude_count} imu={imu_count}"
    )
    return 0


def report_capture(path: Path) -> int:
    """Print machine facts reconstructed from one raw capture."""
    responses: dict[int, bytes] = {}
    frame_count = 0
    manifest = None
    try:
        for record in read_flight_record(path):
            if record.get("type") == "machine":
                manifest = record
            elif record.get("type") == "msp_response":
                direction, command, payload = _response_frame(record)
                if direction == ">":
                    responses[command] = payload
                    frame_count += 1
    except ValueError as error:
        print(f"Invalid flight record: {error}", file=sys.stderr)
        return 2

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


def _mean_and_std(samples: list[tuple[int, int, int]]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    means = tuple(sum(sample[axis] for sample in samples) / len(samples) for axis in range(3))
    deviations = tuple(
        math.sqrt(sum((sample[axis] - means[axis]) ** 2 for sample in samples) / len(samples))
        for axis in range(3)
    )
    return means, deviations


def stillness_report(path: Path) -> int:
    """Report raw stationary-window quality from one flight record."""
    requests = 0
    responses = 0
    damaged = 0
    unpaired = 0
    response_errors = 0
    pending_command = None
    response_times: list[int] = []
    imu_times: list[int] = []
    accel_samples: list[tuple[int, int, int]] = []
    gyro_samples: list[tuple[int, int, int]] = []
    attitude_samples: list[tuple[float, float, float]] = []

    try:
        records = read_flight_record(path)
        for record in records:
            record_type = record.get("type")
            if record_type == "msp_request":
                requests += 1
                try:
                    command = _request_command(record)
                except ValueError:
                    damaged += 1
                    continue
                if pending_command is not None:
                    unpaired += 1
                pending_command = command
            elif record_type == "msp_response":
                responses += 1
                try:
                    direction, command, payload = _response_frame(record)
                except ValueError:
                    damaged += 1
                    continue
                if pending_command != command:
                    unpaired += 1
                pending_command = None
                if direction != ">":
                    response_errors += 1
                    continue
                response_times.append(record["monotonic_time_ns"])
                if command == MSP_RAW_IMU:
                    imu = decode_raw_imu(payload)
                    if imu is not None:
                        accel, gyro = imu
                        accel_samples.append(accel)
                        gyro_samples.append(gyro)
                        imu_times.append(record["monotonic_time_ns"])
                elif command == MSP_ATTITUDE:
                    attitude = decode_attitude(payload)
                    if attitude is not None:
                        attitude_samples.append(attitude)
    except ValueError as error:
        print(f"Invalid flight record: {error}", file=sys.stderr)
        return 2

    if pending_command is not None:
        unpaired += 1
    if not accel_samples or not gyro_samples:
        print("No raw IMU samples in flight record.", file=sys.stderr)
        return 2

    duration_s = (response_times[-1] - response_times[0]) / 1_000_000_000
    imu_duration_s = (imu_times[-1] - imu_times[0]) / 1_000_000_000
    response_hz = (len(response_times) - 1) / duration_s if duration_s > 0.0 else 0.0
    imu_hz = (len(imu_times) - 1) / imu_duration_s if imu_duration_s > 0.0 else 0.0
    worst_imu_gap_ms = max(
        ((right - left) / 1_000_000 for left, right in zip(imu_times, imu_times[1:])),
        default=0.0,
    )
    quiet_start = 0
    quiet_length = 0
    run_start = 0
    for index, gyro in enumerate(gyro_samples):
        if max(abs(value) for value in gyro) <= 8:
            run_length = index - run_start + 1
            if run_length > quiet_length:
                quiet_start, quiet_length = run_start, run_length
        else:
            run_start = index + 1
    quiet_accel = accel_samples[quiet_start : quiet_start + quiet_length]
    quiet_gyro = gyro_samples[quiet_start : quiet_start + quiet_length]
    if not quiet_accel:
        quiet_accel, quiet_gyro = accel_samples, gyro_samples
    accel_mean, accel_std = _mean_and_std(quiet_accel)
    gyro_mean, gyro_std = _mean_and_std(quiet_gyro)
    accel_magnitudes = [math.sqrt(sum(value * value for value in sample)) for sample in quiet_accel]
    accel_magnitude_mean = sum(accel_magnitudes) / len(accel_magnitudes)
    accel_spread = max(abs(value - accel_magnitude_mean) for value in accel_magnitudes)
    accel_variation_percent = 100.0 * accel_spread / accel_magnitude_mean
    gyro_peak = max(abs(value) for sample in gyro_samples for value in sample)
    tilt_spread = 0.0
    if attitude_samples:
        roll_values = [sample[0] for sample in attitude_samples]
        pitch_values = [sample[1] for sample in attitude_samples]
        tilt_spread = max(max(roll_values) - min(roll_values), max(pitch_values) - min(pitch_values))

    calibration_reasons = []
    transport_reasons = []
    if duration_s < 8.0:
        calibration_reasons.append("duration below 8 seconds")
    if quiet_length < 200:
        calibration_reasons.append("fewer than 200 consecutive quiet IMU samples")
    if damaged or unpaired or response_errors:
        calibration_reasons.append("transport errors")
        transport_reasons.append("damaged unpaired or error frames")
    if worst_imu_gap_ms > 100.0:
        transport_reasons.append("IMU gap above 100 ms")
    if accel_variation_percent > 6.0:
        calibration_reasons.append("acceleration variation above 6 percent")
    if tilt_spread > 1.0:
        calibration_reasons.append("tilt variation above 1 degree")

    vector = lambda values: " ".join(f"{value:.3f}" for value in values)
    print(f"duration_s {duration_s:.3f}")
    print(f"requests {requests}")
    print(f"responses {responses}")
    print(f"unpaired {unpaired}")
    print(f"damaged {damaged}")
    print(f"response_errors {response_errors}")
    print(f"response_hz {response_hz:.2f}")
    print(f"imu_samples {len(imu_times)}")
    print(f"quiet_run_samples {quiet_length}")
    print(f"imu_hz {imu_hz:.2f}")
    print(f"worst_imu_gap_ms {worst_imu_gap_ms:.3f}")
    print(f"accel_mean_raw {vector(accel_mean)}")
    print(f"accel_std_raw {vector(accel_std)}")
    print(f"accel_magnitude_mean_raw {accel_magnitude_mean:.3f}")
    print(f"accel_variation_percent {accel_variation_percent:.3f}")
    print(f"gyro_bias_raw {vector(gyro_mean)}")
    print(f"gyro_std_raw {vector(gyro_std)}")
    print(f"gyro_peak_raw {gyro_peak}")
    print(f"tilt_spread_deg {tilt_spread:.3f}")
    print(f"transport_window {'PASS' if not transport_reasons else 'WARN'}")
    for reason in transport_reasons:
        print(f"transport_reason {reason}")
    print(f"calibration_window {'PASS' if not calibration_reasons else 'FAIL'}")
    for reason in calibration_reasons:
        print(f"calibration_reason {reason}")
    return 0 if not calibration_reasons else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only MSP telemetry oracle.")
    parser.add_argument("--device", default=None, help="Serial device path")
    parser.add_argument(
        "--out",
        default=None,
        help="CSV output path (defaults to oracle_<timestamp>.csv)",
    )
    parser.add_argument("--samples", type=int, default=None, help="Max samples before exit")
    parser.add_argument("--seconds", type=float, default=None, help="Max capture duration")
    parser.add_argument("--record-out", default=None, help="Append-only flight record path")
    parser.add_argument("--replay", default=None, help="Replay a raw MSP capture")
    parser.add_argument("--report", default=None, help="Report machine facts from a raw capture")
    parser.add_argument("--stillness", default=None, help="Report stationary raw IMU quality")
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

    if args.stillness:
        capture_path = Path(args.stillness).expanduser().resolve()
        if not capture_path.is_file():
            print(f"Capture not found: {capture_path}", file=sys.stderr)
            return 2
        return stillness_report(capture_path)

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
                run_oracle(
                    device,
                    out_path,
                    max_samples=args.samples,
                    max_seconds=args.seconds,
                    record=record,
                )
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
