from contextlib import ExitStack
import hashlib
from pathlib import Path
import shutil
import tarfile
import tempfile
import zipfile


BASE = Path(__file__).resolve().parent
BINARIES = Path("FlightSim/Binaries/Win64")
SHIPPING = BINARIES / "DCGame-Win64-Shipping.exe"
PAK = Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak")
REQUIRED = (SHIPPING, BINARIES / "dwmapi.dll", BINARIES / "UE4SS.dll", PAK)
VERSIONS = {
    "vq1": {
        "root": Path("AI-GP Simulator v1.0.3391-VQ1/AIGP_VQ1_3391"),
        "legacy": "3a6923f2207a45bf64345b096d2bbd2a789916e32d1fb55beb15417b23003122",
        "hashes": {},
    },
    "vq2": {
        "root": Path("AI-GP Simulator v1.0.3391-VQ2/AIGP_VQ2_3391"),
        "legacy": "3d6527764f43862ad7860694f0783c6f4332eb87b8ccad7bd4c2bb376ce0702e",
        "hashes": {
            SHIPPING: "68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b",
            PAK: "5d424b4ee0de36053914461da56696cfff10c1ed9fab2c6bd883ace58e85883f",
        },
    },
}


def configuration(version):
    return {
        "main.lua": BINARIES / f"Mods/Direct{version.upper()}/Scripts/main.lua",
        "mods.txt": BINARIES / "Mods/mods.txt",
        "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
    }


def prepare(version, base=BASE):
    profile = VERSIONS[version]
    parts = [line.split() for line in (base / "archives/SHA256SUMS").read_text().splitlines()
             if line.strip() and (line.split()[-1] == f"{version}.tar.xz"
                                  or line.split()[-1].startswith(f"{version}-unlocked.tar.gz.part-"))]
    config = {Path("config") / version / name: path for name, path in configuration(version).items()}
    return install(base, version, parts, profile["root"], REQUIRED, config, profile["hashes"],
                   archive_dir=base / "archives", cache_versions=(profile["legacy"],))


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


def install(base, name, parts, archive_root, required, config=None, payload_sha256=None, *,
            archive_dir=None, cache_versions=()):
    if not hasattr(tarfile, "data_filter"):
        raise SystemExit("Use a current Python 3.11 or newer to prepare the simulator.")
    if not parts:
        raise SystemExit(f"No archive parts listed for {name}.")
    config = config or {}
    archive_dir = base if archive_dir is None else archive_dir
    version = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / name
    marker = sim / ".installed"

    try:
        installed = (marker.is_file() and marker.read_text() in (version, *cache_versions)
                     and all((sim / path).is_file() for path in required))
    except (OSError, UnicodeError):
        installed = False
    if installed:
        configure(base, sim, config)
        return sim

    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{name}-install-", dir=runtime) as temporary:
        staging = Path(temporary)
        with ExitStack() as files:
            combined = (files.enter_context(tempfile.TemporaryFile(dir=staging))
                        if len(parts) > 1 else None)
            for expected, filename in parts:
                path = archive_dir / filename
                if not path.is_file():
                    raise SystemExit(f"Missing {filename}. Supply the archive before installing {name}.")
                digest = hashlib.sha256()
                source = files.enter_context(path.open("rb"))
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(block)
                    if combined is not None:
                        combined.write(block)
                if digest.hexdigest() != expected:
                    raise SystemExit(f"Checksum mismatch: {filename}. Download the expected archive and retry.")
                print(f"Verified {filename}", flush=True)
            if combined is None:
                combined = source
            combined.seek(0)
            if zipfile.is_zipfile(combined):
                with zipfile.ZipFile(combined) as archive:
                    archive.extractall(staging)
            else:
                combined.seek(0)
                with tarfile.open(fileobj=combined, mode="r:*") as archive:
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


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Prepare a local simulator installation.")
    parser.add_argument("version", choices=VERSIONS)
    prepare(parser.parse_args().version)
