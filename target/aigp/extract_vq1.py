import hashlib
from pathlib import Path
import tarfile
import tempfile


base = Path(__file__).resolve().parent
sim = base / "AI-GP Simulator v1.0.3391-VQ1" / "AIGP_VQ1_3391"

if (sim / "FlightSim.exe").exists():
    raise SystemExit("VQ1 is already extracted.")

runtime = base / ".runtime"
runtime.mkdir(exist_ok=True)

with tempfile.TemporaryFile(dir=runtime) as combined:
    for line in (base / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split()
        path = base / name

        if not path.is_file():
            raise SystemExit(f"Missing {name}. Run git lfs pull first.")

        digest = hashlib.sha256()

        with path.open("rb") as source:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(block)
                if name.startswith("vq1-unlocked.tar.gz.part-"):
                    combined.write(block)

        if digest.hexdigest() != expected:
            raise SystemExit(f"Checksum mismatch: {name}. Run git lfs pull and retry.")

        print(f"Verified {name}", flush=True)

    combined.seek(0)

    with tarfile.open(fileobj=combined, mode="r:gz") as archive:
        archive.extractall(base, filter="data")

print(f"Extracted VQ1 to {sim}")
