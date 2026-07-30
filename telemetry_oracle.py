#!/usr/bin/env python3
"""Read-only MSP telemetry oracle with console output and CSV logging."""

from __future__ import annotations

import argparse
import csv
import glob
import os
import select
import struct
import sys
import time
import tty
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional

import termios


BAUD = int(os.environ.get("MINIFLIGHT_SERIAL_BAUD", "115200"))
DEVICE_OVERRIDE = os.environ.get("MINIFLIGHT_SERIAL")

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110


READ_ONLY_COMMANDS = {
    MSP_API_VERSION,
    MSP_FC_VARIANT,
    MSP_FC_VERSION,
    MSP_BOARD_INFO,
    MSP_STATUS,
    MSP_RAW_IMU,
    MSP_ATTITUDE,
    MSP_ALTITUDE,
    MSP_ANALOG,
}

SLOW_POLL_SECONDS = {
    MSP_STATUS: 1.0,
    MSP_ANALOG: 1.0,
    MSP_ALTITUDE: 0.5,
}


def xor_checksum(data: Iterable[int]) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def msp_request(command: int) -> bytes:
    if command not in READ_ONLY_COMMANDS:
        raise ValueError(f"MSP command {command} is not in read-only allowlist")
    return b"$M<" + bytes((0, command, command))


class MSPParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)

    def pop(self) -> Optional[tuple[int, bytes]]:
        while True:
            for marker in (b"$M>", b"$M!"):
                start = self.buffer.find(marker)
                if start >= 0:
                    break
            else:
                if len(self.buffer) > 2:
                    del self.buffer[:-2]
                return None

            if start:
                del self.buffer[:start]
            if len(self.buffer) < 6:
                return None

            size = self.buffer[3]
            frame_length = 6 + size
            if len(self.buffer) < frame_length:
                return None

            frame = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            expected = xor_checksum(frame[3:-1])
            if expected != frame[-1]:
                continue

            return frame[4], frame[5:-1]


class MSPSerial:
    def __init__(self, fd: int) -> None:
        self.fd = fd
        self.parser = MSPParser()

    def request(self, command: int, timeout: float = 0.15) -> Optional[bytes]:
        os.write(self.fd, msp_request(command))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.parser.pop()
            if frame is not None:
                response_command, payload = frame
                if response_command != command:
                    continue
                return payload

            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select((self.fd,), (), (), min(0.03, remaining))
            if not readable:
                continue
            chunk = os.read(self.fd, 1024)
            if not chunk:
                raise OSError("serial link closed")
            self.parser.feed(chunk)
        return None


def serial_devices() -> list[str]:
    if DEVICE_OVERRIDE:
        return [DEVICE_OVERRIDE] if os.path.exists(DEVICE_OVERRIDE) else []

    patterns = (
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
    )
    devices: list[str] = []
    for pattern in patterns:
        devices.extend(glob.glob(pattern))
    return sorted(dict.fromkeys(devices), key=lambda path: ("/dev/tty." in path, path))


def open_serial(device: str) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attributes = termios.tcgetattr(fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CREAD | termios.CLOCAL | termios.CS8
        attributes[3] = 0
        attributes[4] = getattr(termios, f"B{BAUD}")
        attributes[5] = getattr(termios, f"B{BAUD}")
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def decode_identity(client: MSPSerial) -> dict[str, Optional[str]]:
    api = client.request(MSP_API_VERSION, timeout=0.35)
    if api is None or len(api) < 3:
        raise RuntimeError("No MSP response for API version")
    variant = client.request(MSP_FC_VARIANT, timeout=0.35)
    version = client.request(MSP_FC_VERSION, timeout=0.35)
    board = client.request(MSP_BOARD_INFO, timeout=0.35)
    return {
        "api_version": f"{api[1]}.{api[2]}" if len(api) >= 3 else None,
        "variant": variant[:4].decode("ascii", "replace").strip("\x00 ") if variant else None,
        "firmware": (
            f"{version[0]}.{version[1]}.{version[2]}"
            if version and len(version) >= 3
            else None
        ),
        "board": board[:4].decode("ascii", "replace").strip("\x00 ") if board else None,
    }


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


def run_oracle(device: str, out_path: Path, max_samples: Optional[int] = None) -> None:
    fd = open_serial(device)
    try:
        client = MSPSerial(fd)
        identity = decode_identity(client)
        print(
            f"Connecting to {device} | variant={identity['variant']} "
            f"board={identity['board']} fw={identity['firmware']} api={identity['api_version']}"
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

                    if command == MSP_ATTITUDE and len(payload) >= 6:
                        roll, pitch, yaw = struct.unpack_from("<hhh", payload)
                        signals["roll"] = roll / 10.0
                        signals["pitch"] = pitch / 10.0
                        signals["yaw"] = float(yaw)
                        attitude_window += 1
                    elif command == MSP_RAW_IMU and len(payload) >= 18:
                        values = struct.unpack_from("<9h", payload)
                        signals["accel"] = values[0:3]
                        signals["gyro"] = values[3:6]
                        mean_count += 1
                        for index in range(3):
                            acc_mean[index] += (values[index] - acc_mean[index]) / mean_count
                            gyr_mean[index] += (values[index + 3] - gyr_mean[index]) / mean_count
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
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only MSP telemetry oracle.")
    parser.add_argument("--device", default=None, help="Serial device path")
    parser.add_argument(
        "--out",
        default=None,
        help="CSV output path (defaults to oracle_<timestamp>.csv)",
    )
    parser.add_argument("--samples", type=int, default=None, help="Max samples before exit")
    args = parser.parse_args()

    if args.device:
        devices = [args.device]
        if not os.path.exists(args.device):
            print(f"Device not found: {args.device}", file=sys.stderr)
            return 2
    else:
        devices = serial_devices()
    if not devices:
        print("No USB serial device found.")
        return 2

    if args.out is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.out = f"telemetry_oracle_{stamp}.csv"

    out_path = Path(args.out).expanduser().resolve()
    print(f"Writing CSV to {out_path}")

    for device in devices:
        try:
            run_oracle(device, out_path, max_samples=args.samples)
            return 0
        except PermissionError:
            print(f"{device} is busy or not permitted. Try closing configurator.")
        except OSError as error:
            print(f"{device} failed: {error}")
        except KeyboardInterrupt:
            print("\nStopped by user.")
            return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
