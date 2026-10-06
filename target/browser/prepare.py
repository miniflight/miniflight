"""Link retained browser runtime assets without copying the original game."""
import argparse
import hashlib
from pathlib import Path


here = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--proof", type=Path, default=here.parent / "aigp/.runtime/browser-proof")
args = parser.parse_args()
proof = args.proof.resolve()
common = proof / "runtime"
patched = proof / "procmem-low"
files = {name: common / name for name in (
    "glibc-rootfs64.zip", "prefix64.zip", "unreal-startup.zip",
    "wine64.zip.manifest.json", "wine64.zip.part000", "wine64.zip.part001", "wine64.zip.part002")}
files.update({name: patched / name for name in ("cpu-fixed-boxedwine64.js", "cpu-fixed-boxedwine64.wasm")})
for name, source in files.items():
    if not source.is_file():
        raise SystemExit("Missing retained browser asset: " + str(source))
wasm = files["cpu-fixed-boxedwine64.wasm"]
if hashlib.sha256(wasm.read_bytes()).hexdigest() != "079bc3037c2df2accec1c1ad569079317d40e362cc05c26596e840605a29bc8b":
    raise SystemExit("Retained browser runtime checksum differs")
destination = here / ".runtime"
destination.mkdir(exist_ok=True)
for name, source in files.items():
    target = destination / name
    if target.is_symlink() and target.resolve() == source:
        continue
    if target.exists() or target.is_symlink():
        raise SystemExit("Will not replace existing runtime asset: " + str(target))
    target.symlink_to(source)
print("Linked 9 browser runtime assets. Original game files stay in their existing folder.")
