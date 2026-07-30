#!/usr/bin/env python3
"""Read-only localhost monitor for a Betaflight flight controller."""

from __future__ import annotations

import os
import signal
import struct
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Optional

from common.serve import RendererServer, SharedState, maybe_launch_browser
from miniflight.msp import (
    MSP_API_VERSION,
    MSP_ANALOG,
    MSP_ATTITUDE,
    MSP_ALTITUDE,
    MSP_BOARD_INFO,
    MSP_BOXIDS,
    MSP_FC_VARIANT,
    MSP_FC_VERSION,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_STATUS,
    MSPParser,
    MSPSerial,
    msp_request,
    open_serial,
    serial_devices,
    xor_checksum,
)
from miniflight.probe import MspMachineProbe
from miniflight.record import FlightRecord, read_flight_record


BAUD = int(os.environ.get("MINIFLIGHT_SERIAL_BAUD", "115200"))
DEVICE_OVERRIDE = os.environ.get("MINIFLIGHT_SERIAL")
HOST = os.environ.get("MINIFLIGHT_CONFIG_HOST", "127.0.0.1")
PORT = int(os.environ.get("MINIFLIGHT_CONFIG_PORT", "8002"))
REPLAY_OVERRIDE = os.environ.get("MINIFLIGHT_REPLAY")

SLOW_POLL_PERIODS = {
    MSP_STATUS: 1.0,
    MSP_ANALOG: 1.0,
    MSP_MOTOR: 0.5,
    MSP_ALTITUDE: 0.5,
}


class LinkState(str, Enum):
    WAITING_USB = "waiting_usb"
    OPENING = "opening"
    HANDSHAKE = "handshake"
    LIVE = "live"
    RETRYING = "retrying"


def stop_signal(_signal_number, _frame) -> None:
    raise KeyboardInterrupt


def empty_snapshot(
    state: LinkState = LinkState.WAITING_USB,
    message: str = "Connect the flight controller by USB-C.",
) -> dict:
    return {
        "connection": {
            "state": state.value,
            "message": message,
            "port": None,
            "last_port": None,
            "baud": BAUD,
            "read_only": True,
            "last_frame_age_ms": None,
            "last_frame_at_ms": None,
        },
        "controller": {
            "variant": None,
            "version": None,
            "api_version": None,
            "board": None,
        },
        "machine": {
            "sensors": [],
            "motor_count": None,
            "has_imu": False,
        },
        "health": {
            "armed": None,
            "cycle_time_us": None,
            "loop_hz": None,
            "cpu_load_percent": None,
            "i2c_errors": None,
            "profile": None,
            "sensors": [],
        },
        "signals": {
            "attitude_deg": [None, None, None],
            "accel_msp": [None, None, None],
            "gyro_msp": [None, None, None],
            "mag_msp": [None, None, None],
            "battery_v": None,
            "current_a": None,
            "mah": None,
            "rssi_percent": None,
            "motors": [],
            "altitude_m": None,
            "vario_mps": None,
        },
        "traffic": {
            "frames_per_second": 0.0,
            "updates_per_second": 0.0,
            "attitude_per_second": 0.0,
            "imu_per_second": 0.0,
            "frames_total": 0,
            "attitude_samples_total": 0,
            "imu_samples_total": 0,
        },
    }


def sensor_names(mask: int) -> list[str]:
    known = (
        (0, "ACC"),
        (1, "BARO"),
        (2, "MAG"),
        (3, "GPS"),
        (4, "RANGE"),
        (5, "GYRO"),
    )
    return [name for bit, name in known if mask & (1 << bit)]


def decode_status(payload: bytes, box_ids: list[int]) -> dict:
    if len(payload) < 11:
        return {}
    cycle_time, i2c_errors, sensors, mode_flags, profile = struct.unpack_from(
        "<HHHIB", payload
    )
    arm_index = next((index for index, box_id in enumerate(box_ids) if box_id == 0), None)
    armed = None if arm_index is None else bool(mode_flags & (1 << arm_index))
    cpu_load = struct.unpack_from("<H", payload, 11)[0] / 10.0 if len(payload) >= 13 else None
    return {
        "armed": armed,
        "cycle_time_us": cycle_time,
        "loop_hz": round(1_000_000 / cycle_time) if cycle_time else None,
        "cpu_load_percent": cpu_load,
        "i2c_errors": i2c_errors,
        "profile": profile + 1,
        "sensors": sensor_names(sensors),
    }


def decode_payload(command: int, payload: bytes, snapshot: dict, box_ids: list[int]) -> None:
    signals = snapshot["signals"]

    if command == MSP_STATUS:
        snapshot["health"].update(decode_status(payload, box_ids))
    elif command == MSP_ATTITUDE and len(payload) >= 6:
        roll, pitch, yaw = struct.unpack_from("<hhh", payload)
        signals["attitude_deg"] = [roll / 10.0, pitch / 10.0, float(yaw)]
    elif command == MSP_RAW_IMU and len(payload) >= 18:
        values = struct.unpack_from("<9h", payload)
        signals["accel_msp"] = list(values[0:3])
        signals["gyro_msp"] = list(values[3:6])
        signals["mag_msp"] = list(values[6:9])
    elif command == MSP_ANALOG and len(payload) >= 7:
        voltage, mah, rssi, current = struct.unpack_from("<BHHH", payload)
        signals["battery_v"] = voltage / 10.0
        signals["mah"] = mah
        signals["rssi_percent"] = round(min(100.0, rssi * 100.0 / 1023.0), 1)
        signals["current_a"] = current / 100.0
    elif command == MSP_MOTOR and len(payload) >= 2:
        count = len(payload) // 2
        motors = list(struct.unpack_from(f"<{count}H", payload))
        while len(motors) > 4 and motors[-1] == 0:
            motors.pop()
        signals["motors"] = motors
    elif command == MSP_ALTITUDE and len(payload) >= 6:
        altitude_cm, vario_cms = struct.unpack_from("<ih", payload)
        signals["altitude_m"] = altitude_cm / 100.0
        signals["vario_mps"] = vario_cms / 100.0


def handshake(client: MSPSerial):
    profile = MspMachineProbe().inspect(client)
    box_payload = client.request(MSP_BOXIDS, timeout=0.25)
    return profile, list(box_payload or b"")


def publish_link(
    shared: SharedState,
    state: LinkState,
    message: str,
    device: Optional[str] = None,
    retain_last_sample: bool = False,
) -> dict:
    snapshot = shared.get_snapshot() if retain_last_sample else empty_snapshot(state, message)
    if "connection" not in snapshot:
        snapshot = empty_snapshot(state, message)

    previous_connection = snapshot["connection"]
    last_port = previous_connection.get("port") or previous_connection.get("last_port")
    last_frame_at_ms = previous_connection.get("last_frame_at_ms")
    age_ms = None
    if isinstance(last_frame_at_ms, (int, float)):
        age_ms = round(max(0.0, time.time() * 1000 - last_frame_at_ms))

    previous_connection.update(
        state=state.value,
        message=message,
        port=device,
        last_port=device or last_port,
        baud=BAUD,
        read_only=True,
        last_frame_age_ms=age_ms,
        last_frame_at_ms=last_frame_at_ms,
    )
    shared.set_snapshot(snapshot)
    return snapshot


def monitor_device(
    shared: SharedState,
    device: str,
    stop_event: threading.Event,
    record: Optional[FlightRecord] = None,
) -> None:
    if record is not None:
        record.link("opening", device)
    publish_link(
        shared,
        LinkState.OPENING,
        "Opening serial link.",
        device,
        retain_last_sample=True,
    )
    fd: Optional[int] = None
    try:
        fd = open_serial(device, BAUD)
        client = MSPSerial(
            fd,
            on_frame=record.response if record is not None else None,
            on_request=record.request if record is not None else None,
        )
        snapshot = publish_link(
            shared,
            LinkState.HANDSHAKE,
            "Reading controller identity.",
            device,
            retain_last_sample=True,
        )
        profile, box_ids = handshake(client)
        snapshot["controller"] = {
            "variant": profile.controller,
            "version": profile.firmware,
            "api_version": profile.api_version,
            "board": profile.board,
        }
        snapshot["machine"] = {
            "sensors": sorted(profile.sensors),
            "motor_count": profile.motor_count,
            "has_imu": profile.has_imu,
        }
        if record is not None:
            record.machine(
                device,
                profile,
                {
                    "attitude": "unused non-IMU slots",
                    "raw_imu": "every second request",
                    "status_s": SLOW_POLL_PERIODS[MSP_STATUS],
                    "analog_s": SLOW_POLL_PERIODS[MSP_ANALOG],
                    "motor_s": SLOW_POLL_PERIODS[MSP_MOTOR],
                    "altitude_s": SLOW_POLL_PERIODS[MSP_ALTITUDE],
                },
            )
        snapshot["connection"].update(
            state=LinkState.LIVE.value,
            message="Live MSP telemetry.",
            port=device,
        )
        shared.set_snapshot(snapshot)
        if record is not None:
            record.link("live", device)

        last_response = time.monotonic()
        rate_window_start = last_response
        rate_window_frames = 0
        rate_window_updates = 0
        rate_window_attitude = 0
        rate_window_imu = 0
        fast_tick = 0
        next_slow_poll = {
            command: last_response + period
            for command, period in SLOW_POLL_PERIODS.items()
        }

        while not stop_event.is_set():
            loop_started = time.monotonic()
            now = loop_started
            imu_slot = fast_tick % 2 == 1
            fast_tick += 1
            if imu_slot:
                command = MSP_RAW_IMU
            else:
                due_slow = [
                    slow_command
                    for slow_command, deadline in next_slow_poll.items()
                    if now >= deadline
                ]
                if due_slow:
                    command = min(due_slow, key=next_slow_poll.get)
                    next_slow_poll[command] = now + SLOW_POLL_PERIODS[command]
                else:
                    command = MSP_ATTITUDE

            payload = client.request(command, timeout=0.05)
            if payload is not None:
                last_response = time.monotonic()
                snapshot["connection"]["last_frame_at_ms"] = round(time.time() * 1000)
                rate_window_frames += 1
                snapshot["traffic"]["frames_total"] += 1
                decode_payload(command, payload, snapshot, box_ids)
                if command == MSP_ATTITUDE:
                    rate_window_attitude += 1
                    snapshot["traffic"]["attitude_samples_total"] += 1
                elif command == MSP_RAW_IMU:
                    rate_window_imu += 1
                    snapshot["traffic"]["imu_samples_total"] += 1

            now = time.monotonic()
            rate_window_updates += 1
            if now - last_response > 1.5:
                raise RuntimeError("MSP telemetry stopped")

            elapsed = now - rate_window_start
            if elapsed >= 1.0:
                snapshot["traffic"]["frames_per_second"] = round(
                    rate_window_frames / elapsed, 1
                )
                snapshot["traffic"]["updates_per_second"] = round(
                    rate_window_updates / elapsed, 1
                )
                snapshot["traffic"]["attitude_per_second"] = round(
                    rate_window_attitude / elapsed, 1
                )
                snapshot["traffic"]["imu_per_second"] = round(
                    rate_window_imu / elapsed, 1
                )
                rate_window_start = now
                rate_window_frames = 0
                rate_window_updates = 0
                rate_window_attitude = 0
                rate_window_imu = 0

            snapshot["connection"]["last_frame_age_ms"] = round(
                (now - last_response) * 1000
            )
            shared.set_snapshot(snapshot)
            stop_event.wait(max(0.0, 0.005 - (time.monotonic() - loop_started)))
    finally:
        if record is not None:
            record.link("closed", device)
        if fd is not None:
            os.close(fd)


def monitor_loop(
    shared: SharedState,
    stop_event: threading.Event,
    record: Optional[FlightRecord] = None,
) -> None:
    last_message = ""
    while not stop_event.is_set():
        devices = serial_devices(DEVICE_OVERRIDE)
        if not devices:
            message = "Connect the flight controller by USB-C."
            if message != last_message:
                publish_link(
                    shared,
                    LinkState.WAITING_USB,
                    message,
                    retain_last_sample=True,
                )
                last_message = message
            stop_event.wait(0.4)
            continue

        connected = False
        for device in devices:
            try:
                monitor_device(shared, device, stop_event, record)
                connected = True
            except PermissionError:
                last_message = "Serial port is busy. Close Betaflight Configurator."
                if record is not None:
                    record.link("retrying", device)
                publish_link(
                    shared,
                    LinkState.RETRYING,
                    last_message,
                    device,
                    retain_last_sample=True,
                )
            except (OSError, RuntimeError) as error:
                last_message = f"{error}. Retrying."
                if record is not None:
                    record.link("retrying", device)
                publish_link(
                    shared,
                    LinkState.RETRYING,
                    last_message,
                    device,
                    retain_last_sample=True,
                )
            if stop_event.is_set():
                break
            stop_event.wait(0.5)
        if not connected:
            stop_event.wait(0.5)


def replay_loop(shared: SharedState, path: Path, stop_event: threading.Event) -> None:
    """Publish recorded MSP responses with their original timing."""
    parser = MSPParser()
    snapshot = empty_snapshot(LinkState.OPENING, "Loading flight record.")
    box_ids: list[int] = []
    first_record_ns: Optional[int] = None
    replay_started = time.monotonic()
    rate_started_ns: Optional[int] = None
    rate_frames = 0
    rate_attitude = 0
    rate_imu = 0

    for record in read_flight_record(path):
        if stop_event.is_set():
            return
        record_ns = record["monotonic_time_ns"]
        if first_record_ns is None:
            first_record_ns = record_ns
            rate_started_ns = record_ns
        due_s = (record_ns - first_record_ns) / 1_000_000_000
        remaining_s = due_s - (time.monotonic() - replay_started)
        if stop_event.wait(max(0.0, remaining_s)):
            return

        record_type = record.get("type")
        if record_type == "machine":
            snapshot["controller"] = {
                "variant": record.get("controller"),
                "version": record.get("firmware"),
                "api_version": record.get("api_version"),
                "board": record.get("board"),
            }
            sensors = list(record.get("sensors") or [])
            snapshot["machine"] = {
                "sensors": sensors,
                "motor_count": record.get("motor_count"),
                "has_imu": "acc" in sensors and "gyro" in sensors,
            }
        elif record_type == "link":
            state = record.get("state")
            snapshot["connection"].update(
                state=LinkState.LIVE.value if state == "live" else LinkState.OPENING.value,
                message="Replaying flight record." if state == "live" else "Loading flight record.",
                port=record.get("device"),
                last_port=record.get("device"),
            )
        elif record_type == "msp_response":
            try:
                parser.feed(bytes.fromhex(record["frame_hex"]))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"invalid MSP response at sequence {record['sequence']}") from error
            while (frame := parser.pop()) is not None:
                direction, command, payload = frame
                if direction != ">":
                    continue
                if command == MSP_BOXIDS:
                    box_ids = list(payload)
                else:
                    decode_payload(command, payload, snapshot, box_ids)
                snapshot["traffic"]["frames_total"] += 1
                rate_frames += 1
                if command == MSP_ATTITUDE:
                    snapshot["traffic"]["attitude_samples_total"] += 1
                    rate_attitude += 1
                elif command == MSP_RAW_IMU:
                    snapshot["traffic"]["imu_samples_total"] += 1
                    rate_imu += 1
                snapshot["connection"]["last_frame_at_ms"] = round(time.time() * 1000)
                snapshot["connection"]["last_frame_age_ms"] = 0

        if rate_started_ns is not None and record_ns - rate_started_ns >= 1_000_000_000:
            elapsed_s = (record_ns - rate_started_ns) / 1_000_000_000
            snapshot["traffic"].update(
                frames_per_second=round(rate_frames / elapsed_s, 1),
                updates_per_second=round(rate_frames / elapsed_s, 1),
                attitude_per_second=round(rate_attitude / elapsed_s, 1),
                imu_per_second=round(rate_imu / elapsed_s, 1),
            )
            rate_started_ns = record_ns
            rate_frames = 0
            rate_attitude = 0
            rate_imu = 0
        shared.set_snapshot(snapshot)

    snapshot["connection"].update(
        state=LinkState.WAITING_USB.value,
        message="Replay complete.",
        port=None,
    )
    shared.set_snapshot(snapshot)
    while not stop_event.wait(0.1):
        last_frame_at_ms = snapshot["connection"].get("last_frame_at_ms")
        if isinstance(last_frame_at_ms, (int, float)):
            snapshot["connection"]["last_frame_age_ms"] = round(
                max(0.0, time.time() * 1000 - last_frame_at_ms)
            )
            shared.set_snapshot(snapshot)


def main() -> None:
    static_dir = Path(__file__).resolve().parent
    shared = SharedState()
    replay_path = Path(REPLAY_OVERRIDE).expanduser() if REPLAY_OVERRIDE else None
    if replay_path is not None and not replay_path.is_file():
        raise SystemExit(f"Flight record not found: {replay_path}")
    publish_link(
        shared,
        LinkState.OPENING if replay_path is not None else LinkState.WAITING_USB,
        "Loading flight record." if replay_path is not None else "Connect the flight controller by USB-C.",
    )

    server = RendererServer(shared, host=HOST, port=PORT, static_dir=static_dir)
    server.start()
    url = f"http://{HOST}:{PORT}".replace("0.0.0.0", "127.0.0.1")
    print(f"Flight Seed is live at {url}")
    print(f"Replaying {replay_path}" if replay_path is not None else "Read-only MSP. Waiting for USB.")
    maybe_launch_browser(
        url,
        env_var="MINIFLIGHT_CONFIG_OPEN_BROWSER",
        default=True,
        delay=0.3,
    )

    record_path = os.environ.get("MINIFLIGHT_RECORD")
    if replay_path is not None and record_path:
        raise SystemExit("MINIFLIGHT_REPLAY and MINIFLIGHT_RECORD cannot be used together")
    record = FlightRecord(Path(record_path).expanduser()) if record_path else None
    if record is not None:
        print(f"Writing flight record to {record.path}")

    stop_event = threading.Event()
    target = replay_loop if replay_path is not None else monitor_loop
    worker_args = (shared, replay_path, stop_event) if replay_path is not None else (shared, stop_event, record)
    worker = threading.Thread(target=target, args=worker_args, daemon=True)
    worker.start()
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if worker.is_alive():
            worker.join(timeout=2.0)
        server.shutdown()
        if record is not None:
            record.close()


if __name__ == "__main__":
    main()
