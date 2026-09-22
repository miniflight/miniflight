import hashlib
from pathlib import Path
import shutil
import tarfile
import tempfile
import zipfile


def configure(base, sim, files):
    for source, relative in files.items():
        destination = sim / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(base / source, destination)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def install(base, name, parts, archive_root, required, config=None, payload_sha256=None):
    if not hasattr(tarfile, "data_filter"):
        raise SystemExit("Use a current Python 3.11 or newer to prepare the simulator.")
    if not parts:
        raise SystemExit(f"No archive parts listed for {name}.")
    config = config or {}
    version = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / name
    marker = sim / ".installed"

    try:
        installed = (marker.is_file() and marker.read_text() == version
                     and all((sim / path).is_file() for path in required))
    except (OSError, UnicodeError):
        installed = False
    if installed:
        configure(base, sim, config)
        return sim

    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{name}-install-", dir=runtime) as temporary:
        staging = Path(temporary)
        with tempfile.TemporaryFile(dir=staging) as combined:
            for expected, filename in parts:
                path = base / filename
                if not path.is_file():
                    raise SystemExit(f"Missing {filename}. Supply the archive before installing {name}.")
                digest = hashlib.sha256()
                with path.open("rb") as source:
                    for block in iter(lambda: source.read(1024 * 1024), b""):
                        digest.update(block)
                        combined.write(block)
                if digest.hexdigest() != expected:
                    raise SystemExit(f"Checksum mismatch: {filename}. Download the expected archive and retry.")
                print(f"Verified {filename}", flush=True)
            combined.seek(0)
            if zipfile.is_zipfile(combined):
                with zipfile.ZipFile(combined) as archive:
                    archive.extractall(staging)
            else:
                combined.seek(0)
                with tarfile.open(fileobj=combined, mode="r:gz") as archive:
                    archive.extractall(staging, filter="data")

        extracted = staging / archive_root
        if not all((extracted / path).is_file() for path in required):
            raise SystemExit(f"{name} archive is missing required simulator files.")
        for relative, expected in (payload_sha256 or {}).items():
            if sha256(extracted / relative) != expected:
                raise SystemExit(f"{name} archive contains the wrong {relative} build.")
        configure(base, extracted, config)
        (extracted / ".installed").write_text(version)
        backup = None
        try:
            if sim.exists() or sim.is_symlink():
                backup = Path(tempfile.mkdtemp(prefix=f"{name}-backup-", dir=runtime)) / name
                sim.rename(backup)
            extracted.rename(sim)
        except BaseException:
            if backup is not None:
                if backup.exists() or backup.is_symlink():
                    backup.rename(sim)
                backup.parent.rmdir()
            raise
        if backup is not None:
            print(f"Preserved previous {name} installation at {backup}", flush=True)
    print(f"Prepared {name} at {sim}", flush=True)
    return sim
