"""Check the browser writer with Python's independent ZIP64 reader."""
import io
import json
from pathlib import Path
import subprocess
import zipfile


fixture = json.loads(subprocess.check_output(
    ["node", str(Path(__file__).with_name("browser_assets.mjs")), "--zip-fixture"], text=True))


class SparseArchive(io.RawIOBase):
    position = 0

    def seek(self, offset, whence=0):
        self.position = offset + (self.position if whence == 1 else fixture["size"] if whence == 2 else 0)
        return self.position

    def tell(self):
        return self.position

    def read(self, size=-1):
        size = min(size if size >= 0 else fixture["size"], fixture["size"] - self.position)
        assert size < 65536, "ZIP reader attempted to materialize the large payload"
        result = bytearray(size)
        for segment in fixture["segments"]:
            data = bytes.fromhex(segment["hex"])
            start = max(self.position, segment["start"])
            end = min(self.position + size, segment["start"] + len(data))
            if start < end:
                result[start - self.position:end - self.position] = data[start - segment["start"]:end - segment["start"]]
        self.position += size
        return bytes(result)


with zipfile.ZipFile(SparseArchive()) as archive:
    prefix = "home/username/aigp/"
    assert archive.namelist() == [prefix + "large.pak", prefix + "after.txt"]
    assert archive.getinfo(prefix + "large.pak").file_size == fixture["largeSize"]
    assert archive.getinfo(prefix + "after.txt").header_offset > 2**32
    assert archive.read(prefix + "after.txt") == b"123456789"
print(json.dumps({"independentZip64Reader": True, "largePayloadAllocated": False}))
