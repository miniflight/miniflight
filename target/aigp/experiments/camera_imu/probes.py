"""The VQ1 test tap's append-only shared memory, drained at Vehicle.read()."""
from enum import Enum, auto
import hashlib
import json
from pathlib import Path
import mmap
import shutil
import struct
import subprocess
import tempfile
import time

from target.aigp.native import BASE


EXPECTED_BINARY_SHA256 = "d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9"
SOURCE = Path(__file__).resolve().parent/"native"


def build():
    """Build version-specific Windows helpers in the generated runtime folder."""
    compiler = shutil.which("x86_64-w64-mingw32-gcc")
    objcopy = shutil.which("x86_64-w64-mingw32-objcopy")
    if compiler is None or objcopy is None:
        raise FileNotFoundError("Install MinGW-w64 with x86_64-w64-mingw32-gcc and x86_64-w64-mingw32-objcopy")
    version = subprocess.check_output([compiler, "--version"], text=True).splitlines()[0]
    hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(SOURCE.iterdir())}
    metadata = dict(compiler=version, sources=hashes,
                    builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    key = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()[:16]
    parent = BASE/".runtime/camera-imu-builds"
    destination = parent/key
    products = ("camera.dll", "imu.dll", "load.exe", "snapshot.bin", "build.json")
    if destination.exists():
        if not all((destination/name).is_file() for name in products):
            raise FileExistsError(f"Incomplete probe build: {destination}")
        cached = json.loads((destination/"build.json").read_text())
        for name, expected in cached["products"].items():
            if hashlib.sha256((destination/name).read_bytes()).hexdigest() != expected:
                raise ValueError(f"Probe cache digest differs: {destination/name}")
        return destination
    parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="probe-build-", dir=parent) as temporary:
        output = Path(temporary)
        subprocess.run([compiler, "-x", "assembler", "-c", str(SOURCE/"snapshot.S"), "-o", str(output/"snapshot.o")], check=True)
        subprocess.run([objcopy, "-O", "binary", "-j", ".text", str(output/"snapshot.o"), str(output/"snapshot.bin")], check=True)
        stub = (output/"snapshot.bin").read_bytes()
        lines = [", ".join(f"0x{byte:02x}" for byte in stub[i:i+16]) for i in range(0, len(stub), 16)]
        (output/"snapshot.h").write_text("static unsigned char stub[] = {\n    "+",\n    ".join(lines)+"\n};\n")
        flags = [compiler, "-O2", "-Wall", "-Wextra"]
        for name in ("camera", "imu"):
            subprocess.run([*flags, "-shared", "-I", str(output), str(SOURCE/f"{name}.c"), "-o", str(output/f"{name}.dll")], check=True)
        subprocess.run([*flags, str(SOURCE/"load.c"), "-o", str(output/"load.exe")], check=True)
        metadata["products"] = {name: hashlib.sha256((output/name).read_bytes()).hexdigest() for name in products[:-1]}
        (output/"build.json").write_text(json.dumps(metadata, indent=2)+"\n")
        try:
            output.rename(destination)
        except OSError:
            if not all((destination/name).is_file() for name in products):
                raise
    return destination


class CapturePhase(Enum):
    WAITING_FOR_SAMPLE = auto()
    DRAINING = auto()
    FAULTED = auto()
    CLOSED = auto()


class ClockProbePhase(Enum):
    UNCALIBRATED = auto()
    WAITING_FOR_ACK = auto()
    ACKNOWLEDGED = auto()
    CALIBRATED = auto()
    FAILED = auto()


class Tap:
    header = struct.Struct("<IIIIQQ")
    record = struct.Struct("<QQfI6fQ")
    capacity = 32768

    def __init__(self, path):
        with Path(path).open("r+b") as file:
            self.mapping = mmap.mmap(file.fileno(), 0)
        self.phase = CapturePhase.WAITING_FOR_SAMPLE
        self.clock_phase = ClockProbePhase.UNCALIBRATED
        if len(self.mapping) != self.header.size+self.capacity*self.record.size:
            self.close()
            raise RuntimeError("invalid IMU capture mapping")
        self.frequency = self.header.unpack_from(self.mapping)[4]
        if not self.frequency:
            self.close()
            raise RuntimeError("invalid IMU capture mapping")
        self.discard()

    def close(self):
        if self.phase is not CapturePhase.CLOSED:
            self.mapping.close()
            self.phase = CapturePhase.CLOSED

    def require_open(self):
        if self.phase is CapturePhase.CLOSED:
            raise RuntimeError("producer IMU tap is closed")

    def wait_for_producer(self, process, timeout=45):
        self.require_open()
        deadline = time.monotonic() + timeout
        while struct.unpack_from("<I", self.mapping)[0] == 0:
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("simulator did not publish full IMU samples")
            time.sleep(.02)

    def discard(self):
        self.require_open()
        self.index = struct.unpack_from("<I", self.mapping)[0]
        self.previous = None
        self.phase = CapturePhase.WAITING_FOR_SAMPLE

    def calibrate(self, count=20):
        self.require_open()
        intervals = []
        seq = self.header.unpack_from(self.mapping)[1]
        for _ in range(count):
            seq += 1
            start = time.monotonic_ns()
            struct.pack_into("<I", self.mapping, 4, seq)
            self.clock_phase = ClockProbePhase.WAITING_FOR_ACK
            while self.clock_phase is ClockProbePhase.WAITING_FOR_ACK:
                _, _, ack, _, _, ticks = self.header.unpack_from(self.mapping)
                now = time.monotonic_ns()
                if ack == seq:
                    counter = ticks*10**9//self.frequency
                    intervals.append((start-counter, now-counter))
                    self.clock_phase = ClockProbePhase.ACKNOWLEDGED
                elif now-start > 1_000_000_000:
                    self.clock_phase = ClockProbePhase.FAILED
                    raise TimeoutError("producer did not answer its clock probe")
                else:
                    time.sleep(.0001)
        lower = max(a for a, _ in intervals)
        upper = min(b for _, b in intervals)
        if lower > upper:
            self.clock_phase = ClockProbePhase.FAILED
            raise RuntimeError("IMU clock offset bounds do not overlap")
        self.clock_phase = ClockProbePhase.CALIBRATED
        return lower, upper

    def read(self):
        self.require_open()
        if self.phase is CapturePhase.FAULTED:
            raise RuntimeError("discard the failed producer IMU epoch before reading again")
        count = struct.unpack_from("<I", self.mapping)[0]
        if not self.index <= count < self.capacity:
            self.phase = CapturePhase.FAULTED
            raise RuntimeError("producer IMU buffer exhausted or reset")
        start = self.header.size + self.index*self.record.size
        end = self.header.size + count*self.record.size
        batch = self.mapping[start:end]
        read_ns = time.monotonic_ns()
        rows = list(self.record.iter_unpack(batch))
        previous, phase = self.previous, self.phase
        for stamp, source, dt, slot, *tail in rows:
            ticks = tail[-1]
            if phase is CapturePhase.DRAINING:
                t, src, old_slot, clock = previous
                if stamp <= t or source != src or (slot-old_slot)%8 != 1 or ticks <= clock:
                    self.phase = CapturePhase.FAULTED
                    raise RuntimeError(f"noncontiguous producer IMU: {previous} -> {(stamp, source, slot, ticks)}")
            if not 0 < dt < .1:
                self.phase = CapturePhase.FAULTED
                raise RuntimeError("invalid producer integration interval")
            previous = stamp, source, slot, ticks
            phase = CapturePhase.DRAINING
        self.index, self.previous, self.phase = count, previous, phase
        return dict(read_ns=read_ns, rows=rows)
