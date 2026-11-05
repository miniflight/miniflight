"""One read-only MSP request at a time on a USB serial port."""
from contextlib import contextmanager
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


def pop_reply(buffer):
    while True:
        starts = [i for prefix in (b"$M>", b"$M!") if (i := buffer.find(prefix)) >= 0]
        if not starts:
            del buffer[:-2]
            return None
        del buffer[:min(starts)]
        if len(buffer) < 6:
            return None
        length, start = buffer[3], 5
        if length == 255:
            if len(buffer) < 7:
                return None
            length, start = int.from_bytes(buffer[5:7], "little"), 7
        end = start + length + 1
        if len(buffer) < end:
            return None
        frame = bytes(buffer[:end])
        del buffer[:end]
        if checksum(frame[3:-1]) == frame[-1]:
            return frame[2] == ord(">"), frame[4], frame[start:-1]


class MSP:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()

    def read(self, command, timeout=.3, *, payload=b""):
        if command not in READS:
            raise ValueError(f"Unsupported MSP read command {command}")
        if command == FLASH_READ and len(payload) != 7 or command != FLASH_READ and payload:
            raise ValueError(f"Invalid MSP {command} request length")
        body = bytes((len(payload), command)) + payload
        request = b"$M<" + body + bytes((checksum(body),))
        if os.write(self.fd, request) != len(request):
            raise OSError("Incomplete MSP request")
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            while (reply := pop_reply(self.buffer)) is not None:
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


@contextmanager
def connect(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[:6] = [0, 0, termios.CREAD | termios.CLOCAL | termios.CS8, 0, termios.B115200, termios.B115200]
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield MSP(fd)
    finally:
        os.close(fd)
