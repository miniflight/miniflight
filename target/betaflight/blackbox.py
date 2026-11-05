"""Read and validate the existing Blackbox flash bytes."""
import struct

from target.betaflight import msp


def summary(link):
    flags, _, capacity, used = struct.unpack("<BIII", link.read(msp.FLASH_SUMMARY))
    if used > capacity:
        raise ValueError("Invalid Blackbox flash size")
    return dict(ready=flags & 3 == 3, capacity=capacity, used=used)


def blocks(link, size):
    address = 0
    while address < size:
        requested = min(4096, size-address)
        reply = link.read(msp.FLASH_READ, timeout=1, payload=struct.pack("<IHB", address, requested, 0))
        returned, count, compression = struct.unpack_from("<IHB", reply)
        data = reply[7:]
        if returned != address or compression or len(data) != count or not 0 < count <= requested:
            raise OSError(f"Invalid Blackbox block at {address}")
        yield data
        address += count
