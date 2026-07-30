#!/usr/bin/env python3
"""Read-only localhost monitor for a Betaflight flight controller."""

from __future__ import annotations

import glob
import os
import select
import struct
import termios
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Iterable, Optional

from common.serve import RendererServer, SharedState, maybe_launch_browser
from miniflight.probe import (
    MSP_API_VERSION,
    MSP_BOARD_INFO,
    MSP_FC_VARIANT,
    MSP_FC_VERSION,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_STATUS,
    MspMachineProbe,
)


BAUD = int(os.environ.get("MINIFLIGHT_SERIAL_BAUD", "115200"))
DEVICE_OVERRIDE = os.environ.get("MINIFLIGHT_SERIAL")
HOST = os.environ.get("MINIFLIGHT_CONFIG_HOST", "127.0.0.1")
PORT = int(os.environ.get("MINIFLIGHT_CONFIG_PORT", "8002"))

MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_BOXIDS = 119

READ_ONLY_COMMANDS = frozenset(
    {
        MSP_API_VERSION,
        MSP_FC_VARIANT,
        MSP_FC_VERSION,
        MSP_BOARD_INFO,
        MSP_STATUS,
        MSP_RAW_IMU,
        MSP_MOTOR,
        MSP_ATTITUDE,
        MSP_ALTITUDE,
        MSP_ANALOG,
        MSP_BOXIDS,
    }
)

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


def xor_checksum(data: Iterable[int]) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def msp_request(command: int) -> bytes:
    """Build one read-only MSP v1 request."""
    if command not in READ_ONLY_COMMANDS:
        raise ValueError(f"MSP command {command} is not in the read-only allowlist")
    return b"$M<" + bytes((0, command, command))


class MSPParser:
    """Incrementally parse MSP v1 response frames."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)

    def pop(self) -> Optional[tuple[str, int, bytes]]:
        while True:
            starts = [
                index
                for marker in (b"$M>", b"$M!")
                if (index := self.buffer.find(marker)) >= 0
            ]
            if not starts:
                if len(self.buffer) > 2:
                    del self.buffer[:-2]
                return None

            start = min(starts)
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

            direction = chr(frame[2])
            command = frame[4]
            payload = frame[5:-1]
            return direction, command, payload


class MSPSerial:
    """Small synchronous MSP client over one serial file descriptor."""

    def __init__(self, fd: int) -> None:
        self.fd = fd
        self.parser = MSPParser()

    def request(self, command: int, timeout: float = 0.15) -> Optional[bytes]:
        os.write(self.fd, msp_request(command))
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            frame = self.parser.pop()
            if frame is not None:
                direction, response_command, payload = frame
                if response_command != command:
                    continue
                return payload if direction == ">" else None

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


def monitor_device(shared: SharedState, device: str, stop_event: threading.Event) -> None:
    publish_link(
        shared,
        LinkState.OPENING,
        "Opening serial link.",
        device,
        retain_last_sample=True,
    )
    fd = open_serial(device)
    try:
        client = MSPSerial(fd)
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
        snapshot["connection"].update(
            state=LinkState.LIVE.value,
            message="Live MSP telemetry.",
            port=device,
        )
        shared.set_snapshot(snapshot)

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
            attitude_slot = fast_tick % 2 == 0
            fast_tick += 1
            if attitude_slot:
                command = MSP_ATTITUDE
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
                    command = MSP_RAW_IMU

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
        os.close(fd)


def monitor_loop(shared: SharedState, stop_event: threading.Event) -> None:
    last_message = ""
    while not stop_event.is_set():
        devices = serial_devices()
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
                monitor_device(shared, device, stop_event)
                connected = True
            except PermissionError:
                last_message = "Serial port is busy. Close Betaflight Configurator."
                publish_link(
                    shared,
                    LinkState.RETRYING,
                    last_message,
                    device,
                    retain_last_sample=True,
                )
            except (OSError, RuntimeError) as error:
                last_message = f"{error}. Retrying."
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


def main() -> None:
    static_dir = Path(__file__).resolve().parent
    shared = SharedState()
    publish_link(shared, LinkState.WAITING_USB, "Connect the flight controller by USB-C.")

    server = RendererServer(shared, host=HOST, port=PORT, static_dir=static_dir)
    server.start()
    url = f"http://{HOST}:{PORT}".replace("0.0.0.0", "127.0.0.1")
    print(f"Flight Seed is live at {url}")
    print("Read-only MSP. Waiting for USB.")
    maybe_launch_browser(
        url,
        env_var="MINIFLIGHT_CONFIG_OPEN_BROWSER",
        default=True,
        delay=0.3,
    )

    stop_event = threading.Event()
    worker = threading.Thread(target=monitor_loop, args=(shared, stop_event), daemon=True)
    worker.start()
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


if __name__ == "__main__":
    main()
