"""One read-only MSP request at a time on a USB serial port."""
from contextlib import contextmanager
from enum import Enum, auto
import os
import select
import termios
import time

API_VERSION, FC_VARIANT, FC_VERSION, BOARD_INFO = 1, 2, 3, 4
STATUS, RAW_IMU, MOTOR, ATTITUDE, ALTITUDE, ANALOG, BOX_IDS = 101, 102, 104, 108, 109, 110, 119
FLASH_SUMMARY, FLASH_READ = 70, 71
READS = frozenset((API_VERSION, FC_VARIANT, FC_VERSION, BOARD_INFO, STATUS, RAW_IMU, MOTOR,
                   ATTITUDE, ALTITUDE, ANALOG, BOX_IDS, FLASH_SUMMARY, FLASH_READ))


def checksum(data):
    result = 0
    for byte in data:
        result ^= byte
    return result


class ReplyPhase(Enum):
    SEARCHING = auto()
    READING_LENGTH = auto()
    READING_BODY = auto()


class ReplyParser:
    def __init__(self, buffer):
        self.buffer = buffer
        self.phase = ReplyPhase.SEARCHING

    def pop(self):
        buffer = self.buffer
        while True:
            if self.phase is ReplyPhase.SEARCHING:
                starts = [i for prefix in (b"$M>", b"$M!") if (i := buffer.find(prefix)) >= 0]
                if not starts:
                    del buffer[:-2]
                    return None
                del buffer[:min(starts)]
                self.phase = ReplyPhase.READING_LENGTH
            if self.phase is ReplyPhase.READING_LENGTH:
                if len(buffer) < 6:
                    return None
                length, start = buffer[3], 5
                if length == 255:
                    if len(buffer) < 7:
                        return None
                    length, start = int.from_bytes(buffer[5:7], "little"), 7
                self.start, self.end = start, start + length + 1
                self.phase = ReplyPhase.READING_BODY
            if len(buffer) < self.end:
                return None
            frame = bytes(buffer[:self.end])
            del buffer[:self.end]
            self.phase = ReplyPhase.SEARCHING
            if checksum(frame[3:-1]) == frame[-1]:
                return frame[2] == ord(">"), frame[4], frame[self.start:-1]


def pop_reply(buffer):
    return ReplyParser(buffer).pop()


class RequestPhase(Enum):
    IDLE = auto()
    AWAITING_REPLY = auto()
    CLOSED = auto()


class MSP:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()
        self.parser = ReplyParser(self.buffer)
        self.phase = RequestPhase.IDLE

    def read(self, command, timeout=.3, *, payload=b""):
        if self.phase is RequestPhase.CLOSED:
            raise OSError("USB serial closed")
        if self.phase is RequestPhase.AWAITING_REPLY:
            raise OSError("An MSP request is already waiting for a reply")
        if command not in READS:
            raise ValueError(f"Unsupported MSP read command {command}")
        if command == FLASH_READ and len(payload) != 7 or command != FLASH_READ and payload:
            raise ValueError(f"Invalid MSP {command} request length")
        body = bytes((len(payload), command)) + payload
        request = b"$M<" + body + bytes((checksum(body),))
        self.phase = RequestPhase.AWAITING_REPLY
        try:
            if os.write(self.fd, request) != len(request):
                raise OSError("Incomplete MSP request")
            return self._await_reply(command, timeout)
        finally:
            if self.phase is RequestPhase.AWAITING_REPLY:
                self.phase = RequestPhase.IDLE

    def _await_reply(self, command, timeout):
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            while (reply := self.parser.pop()) is not None:
                ok, response_command, data = reply
                if response_command == command:
                    if not ok:
                        raise OSError(f"Flight controller rejected MSP {command}")
                    return data
            if select.select([self.fd], [], [], remaining)[0]:
                data = os.read(self.fd, 4096)
                if not data:
                    raise OSError("USB serial closed")
                self.buffer.extend(data)
        raise TimeoutError(f"No reply to MSP {command}")

    def close(self):
        if self.phase is not RequestPhase.CLOSED:
            self.phase = RequestPhase.CLOSED
            os.close(self.fd)


@contextmanager
def connect(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    link = MSP(fd)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[:6] = [0, 0, termios.CREAD | termios.CLOCAL | termios.CS8, 0, termios.B115200, termios.B115200]
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield link
    finally:
        link.close()
