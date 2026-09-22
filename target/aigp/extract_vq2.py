from pathlib import Path

if __package__:
    from .runtime import install
else:
    from runtime import install


BASE = Path(__file__).resolve().parent
ARCHIVE_ROOT = Path("AI-GP Simulator v1.0.3391-VQ2/AIGP_VQ2_3391")
BINARIES = Path("FlightSim/Binaries/Win64")
SHIPPING = BINARIES / "DCGame-Win64-Shipping.exe"
PAK = Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak")
REQUIRED = (
    SHIPPING,
    BINARIES / "dwmapi.dll",
    BINARIES / "UE4SS.dll",
    PAK,
)
PAYLOAD_SHA256 = {
    SHIPPING: "68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b",
    PAK: "5d424b4ee0de36053914461da56696cfff10c1ed9fab2c6bd883ace58e85883f",
}
CONFIG = {
    "main.lua": BINARIES / "Mods/DirectVQ2/Scripts/main.lua",
    "mods.txt": BINARIES / "Mods/mods.txt",
    "UE4SS-settings.ini": BINARIES / "UE4SS-settings.ini",
}


def prepare(base=BASE):
    parts = [line.split() for line in (base / "SHA256SUMS").read_text().splitlines()
             if line.strip() and line.split()[-1].startswith("vq2-unlocked.tar.gz.part-")]
    config = {Path("vq2") / name: path for name, path in CONFIG.items()}
    return install(base, "vq2", parts, ARCHIVE_ROOT, REQUIRED, config, PAYLOAD_SHA256)


if __name__ == "__main__":
    prepare()
