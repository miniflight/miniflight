from pathlib import Path

if __package__:
    from .runtime import install
else:
    from runtime import install


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


def prepare(base=BASE):
    parts = [line.split() for line in (base / "SHA256SUMS").read_text().splitlines()
             if line.strip() and line.split()[-1].startswith("vq1-unlocked.tar.gz.part-")]
    config = {Path("vq1") / name: path for name, path in CONFIG.items()}
    return install(base, "vq1", parts, ARCHIVE_ROOT, REQUIRED, config)


if __name__ == "__main__":
    prepare()
