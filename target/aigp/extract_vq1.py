import hashlib
from pathlib import Path
import shutil
import tarfile
import tempfile


BASE = Path(__file__).resolve().parent
ARCHIVE_ROOT = Path("AI-GP Simulator v1.0.3391-VQ1/AIGP_VQ1_3391")
BINARIES = Path("FlightSim/Binaries/Win64")
REQUIRED = (
    BINARIES / "DCGame-Win64-Shipping.exe",
    BINARIES / "dwmapi.dll",
    BINARIES / "UE4SS.dll",
    Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak"),
)
CONFIG = {
    "main.lua": BINARIES / "Mods/DirectVQ1/Scripts/main.lua",
    "mods.txt": BINARIES / "Mods/mods.txt",
    "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
}


def configure(base, sim):
    for name, relative in CONFIG.items():
        destination = sim / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(base / "vq1" / name, destination)


def prepare(base=BASE):
    if not hasattr(tarfile, "data_filter"):
        raise SystemExit("Use a current Python 3.11 or newer to prepare VQ1.")

    parts = [line.split() for line in (base / "SHA256SUMS").read_text().splitlines()
             if line.strip() and line.split()[-1].startswith("vq1-unlocked.tar.gz.part-")]
    if not parts:
        raise SystemExit("No VQ1 archive parts listed in SHA256SUMS.")
    version = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / "vq1"
    marker = sim / ".installed"

    if sim.exists():
        if (not marker.is_file() or marker.read_text() != version
                or not all((sim / path).is_file() for path in REQUIRED)):
            raise SystemExit(f"Incomplete or different VQ1 installation: {sim}. Move it aside and retry.")
        configure(base, sim)
        return sim

    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="vq1-install-", dir=runtime) as temporary:
        staging = Path(temporary)
        with tempfile.TemporaryFile(dir=staging) as combined:
            for expected, name in parts:
                path = base / name
                if not path.is_file():
                    raise SystemExit(f"Missing {name}. Run git lfs pull first.")
                digest = hashlib.sha256()
                with path.open("rb") as source:
                    for block in iter(lambda: source.read(1024 * 1024), b""):
                        digest.update(block)
                        combined.write(block)
                if digest.hexdigest() != expected:
                    raise SystemExit(f"Checksum mismatch: {name}. Run git lfs pull and retry.")
                print(f"Verified {name}", flush=True)
            combined.seek(0)
            with tarfile.open(fileobj=combined, mode="r:gz") as archive:
                archive.extractall(staging, filter="data")

        extracted = staging / ARCHIVE_ROOT
        if not all((extracted / path).is_file() for path in REQUIRED):
            raise SystemExit("VQ1 archive is missing required simulator files.")
        configure(base, extracted)
        (extracted / ".installed").write_text(version)
        extracted.rename(sim)
    print(f"Prepared VQ1 at {sim}", flush=True)
    return sim


if __name__ == "__main__":
    prepare()
