"""Install and own the unchanged native AI-GP process."""

from contextlib import contextmanager
import fcntl
import hashlib
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile


BASE = Path(__file__).resolve().parent
BINARIES = Path("FlightSim/Binaries/Win64")
SHIPPING = BINARIES / "DCGame-Win64-Shipping.exe"
PAK = Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak")
REQUIRED = (SHIPPING, BINARIES / "dwmapi.dll", BINARIES / "UE4SS.dll", PAK)
VERSIONS = {
    "vq1": (Path("AI-GP Simulator v1.0.3391-VQ1/AIGP_VQ1_3391"),
            "3a6923f2207a45bf64345b096d2bbd2a789916e32d1fb55beb15417b23003122"),
    "vq2": (Path("AI-GP Simulator v1.0.3391-VQ2/AIGP_VQ2_3391"),
            "3d6527764f43862ad7860694f0783c6f4332eb87b8ccad7bd4c2bb376ce0702e"),
}

TARGETS = {
    "vq1.r1": ("vq1", "r1", "MAP_anduril_master"),
    "vq2.r1": ("vq2", "r1", "MAP_anduril_master"),
    "vq2.r2": ("vq2", "r2", "MAP_arsenal_master"),
}


def prepare(version, base=BASE):
    """Verify and extract the selected tar archive once, then copy its three config files."""
    root, legacy_stamp = VERSIONS[version]
    parts = [line.split() for line in (base / "archives/SHA256SUMS").read_text().splitlines()
             if line.strip() and line.split()[-1] == f"{version}.tar.xz"]
    expected, filename = parts[0]
    stamp = hashlib.sha256(repr(parts).encode()).hexdigest()
    runtime = base / ".runtime"
    sim = runtime / version
    marker = sim / ".installed"
    installed = (marker.is_file() and marker.read_text() in (stamp, legacy_stamp)
                 and all((sim / path).is_file() for path in REQUIRED))
    if not installed:
        if sim.exists():
            raise FileExistsError(f"Incomplete {sim}; move it aside before preparing again")
        archive = base / "archives" / filename
        if sha256(archive) != expected:
            raise ValueError(f"Checksum mismatch: {filename}")
        runtime.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=f"{version}-install-", dir=runtime) as temporary:
            with tarfile.open(archive, "r:xz") as source:
                source.extractall(temporary, filter="data")
            extracted = Path(temporary) / root
            if not all((extracted / path).is_file() for path in REQUIRED):
                raise ValueError(f"{filename} is missing required simulator files")
            (extracted / ".installed").write_text(stamp)
            extracted.rename(sim)
    for name, relative in {
        "main.lua": BINARIES / f"Mods/Direct{version.upper()}/Scripts/main.lua",
        "mods.txt": BINARIES / "Mods/mods.txt",
        "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
    }.items():
        destination = sim / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(base / "config" / version / name, destination)
    return sim


def sha256(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def wine_commands():
    if sys.platform not in ("darwin", "linux"):
        raise RuntimeError("the runner requires macOS or Linux")
    configured = os.environ.get("WINE")
    default = ("/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/wine64"
               if sys.platform == "darwin" else "wine64")
    wine = shutil.which(configured or default)
    if wine is None and not configured and sys.platform == "linux":
        wine = shutil.which("wine")
    if wine is None:
        raise FileNotFoundError("Wine not found; install it or set WINE to its executable")
    server = os.environ.get("WINESERVER", str(Path(wine).with_name("wineserver")))
    server = shutil.which(server)
    if server is None and not configured and "WINESERVER" not in os.environ and sys.platform == "linux":
        server = shutil.which("wineserver")
    if server is None:
        raise FileNotFoundError("wineserver not found; set WINESERVER to the matching executable")
    return wine, server


@contextmanager
def launch(target, simulator_args=(), attach=False):
    """Own one Wine prefix and process group, or leave an attached process alone."""
    if target not in TARGETS:
        raise ValueError(f"unsupported simulator: {target}")
    if attach:
        yield None
        return
    # Refuse an existing simulator before touching its Wine prefix.
    for port in (14560, 5601):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind(("127.0.0.1", port))
            except OSError as error:
                raise OSError(f"UDP {port} is in use; stop the existing simulator first") from error
    wine, server = wine_commands()
    version, mode, level = TARGETS[target]
    prefix = BASE / ".runtime" / f"{version}-wine"
    prefix.mkdir(parents=True, exist_ok=True)
    with (prefix / ".runner.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f"{version} is already running") from None
        sim = prepare(version, BASE)
        env = dict(os.environ, WINEPREFIX=str(prefix), WINEDLLOVERRIDES="dwmapi=n,b;winegstreamer=")
        if version == "vq2":
            env["MINIFLIGHT_VQ2_MODE"] = mode
        if sys.platform == "darwin":
            env["WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER"] = "1"
        command = [wine, str(sim / SHIPPING), f"/Game/levelsMaster/{level}?game=/Script/DCGame.GameModeRaceBase",
                   "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-NoSplash", *simulator_args]
        def stop_wine():
            stopped = subprocess.run([server, "-k"], env=env, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL, timeout=10)
            if stopped.returncode not in (0, 1):  # Wine returns 1 when no server is running.
                stopped.check_returncode()

        process = None
        try:
            stop_wine()
            print(f"{target}: starting", flush=True)
            process = subprocess.Popen(command, cwd=sim, env=env, start_new_session=True)
            yield process
        finally:
            # Also clean up when Wine starts children but its launcher exits.
            try:
                stop_wine()
            finally:
                _stop_process(process)
                if process is not None:
                    print(f"{target}: stopped", flush=True)


def _stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass  # The process group exited before the kill.
        process.wait(timeout=5)
