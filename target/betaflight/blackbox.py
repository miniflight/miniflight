"""Read and validate the existing Blackbox flash bytes."""
from dataclasses import dataclass
from enum import Enum, auto
import struct

from target.betaflight import msp


def summary(link):
    flags, _, capacity, used = struct.unpack("<BIII", link.read(msp.FLASH_SUMMARY))
    if used > capacity:
        raise ValueError("Invalid Blackbox flash size")
    return dict(ready=flags & 3 == 3, capacity=capacity, used=used)


class FlashPhase(Enum):
    READING = auto()
    COMPLETE = auto()


@dataclass
class FlashTransfer:
    size: int
    address: int = 0

    @property
    def phase(self):
        return FlashPhase.READING if self.address < self.size else FlashPhase.COMPLETE

    def read_block(self, link):
        requested = min(4096, self.size-self.address)
        reply = link.read(msp.FLASH_READ, timeout=1, payload=struct.pack("<IHB", self.address, requested, 0))
        returned, count, compression = struct.unpack_from("<IHB", reply)
        data = reply[7:]
        if returned != self.address or compression or len(data) != count or not 0 < count <= requested:
            raise OSError(f"Invalid Blackbox block at {self.address}")
        self.address += count
        return data


def blocks(link, size):
    transfer = FlashTransfer(size)
    while transfer.phase is FlashPhase.READING:
        yield transfer.read_block(link)
