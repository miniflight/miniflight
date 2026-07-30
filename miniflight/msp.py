"""Read-only MSP v1 transport for Miniflight tools."""

from __future__ import annotations

import glob
import os
import select
import termios
import time
from typing import Callable, Iterable, Optional


MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
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
        self.last_frame: Optional[bytes] = None

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
            if xor_checksum(frame[3:-1]) != frame[-1]:
                continue

            self.last_frame = frame
            return chr(frame[2]), frame[4], frame[5:-1]


class MSPSerial:
    """Small synchronous MSP client over one serial file descriptor."""

    def __init__(self, fd: int, on_frame: Optional[Callable[[bytes], None]] = None) -> None:
        self.fd = fd
        self.parser = MSPParser()
        self.on_frame = on_frame

    def request(self, command: int, timeout: float = 0.15) -> Optional[bytes]:
        os.write(self.fd, msp_request(command))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self.parser.pop()
            if response is not None:
                direction, response_command, payload = response
                if self.on_frame is not None and self.parser.last_frame is not None:
                    self.on_frame(self.parser.last_frame)
                if response_command == command:
                    return payload if direction == ">" else None

            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select((self.fd,), (), (), min(0.03, remaining))
            if readable:
                chunk = os.read(self.fd, 1024)
                if not chunk:
                    raise OSError("serial link closed")
                self.parser.feed(chunk)
        return None


def serial_devices(override: Optional[str] = None) -> list[str]:
    if override:
        return [override] if os.path.exists(override) else []
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


def open_serial(device: str, baud: int) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attributes = termios.tcgetattr(fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CREAD | termios.CLOCAL | termios.CS8
        attributes[3] = 0
        attributes[4] = getattr(termios, f"B{baud}")
        attributes[5] = getattr(termios, f"B{baud}")
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise
