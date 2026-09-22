from contextlib import redirect_stdout
import hashlib
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from target.aigp import extract_vq1, runtime


REQUIRED = (
    Path("FlightSim.exe"),
    Path("FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"),
    Path("FlightSim/Content/Paks/FlightSim-WindowsNoEditor.pak"),
)


class AIGPRuntimeTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.enterContext(redirect_stdout(io.StringIO()))

    def archive(self, name, root=Path(".")):
        path = self.base / f"{name}.zip"
        with zipfile.ZipFile(path, "w") as archive:
            for required in REQUIRED:
                archive.writestr(str(root / required), f"{name}: {required}")
        return [[hashlib.sha256(path.read_bytes()).hexdigest(), path.name]]

    def cached_vq1(self):
        parts = [["a" * 64, "vq1-unlocked.tar.gz.part-aa"],
                 ["b" * 64, "vq1-unlocked.tar.gz.part-ab"]]
        sim = self.base / ".runtime/vq1"
        for required in extract_vq1.REQUIRED:
            path = sim / required
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"existing vq1")
        (sim / ".installed").write_text(hashlib.sha256(repr(parts).encode()).hexdigest())
        return parts, sim

    def test_plain_vq2_zip_needs_no_mods(self):
        parts = self.archive("vq2")
        sim = runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
        self.assertEqual(sim, self.base / ".runtime/vq2")
        for required in REQUIRED:
            self.assertEqual((sim / required).read_text(), f"vq2: {required}")
        self.assertEqual({path.relative_to(sim) for path in sim.rglob("*") if path.is_file()},
                         {*REQUIRED, Path(".installed")})
        self.assertEqual(list(sim.parent.iterdir()), [sim])

    def test_vq2_failure_preserves_existing_vq1(self):
        _, vq1 = self.cached_vq1()
        before = {path.relative_to(vq1): path.read_bytes()
                  for path in vq1.rglob("*") if path.is_file()}
        parts = self.archive("vq2")
        (self.base / parts[0][1]).write_bytes(b"incomplete download")
        with self.assertRaisesRegex(SystemExit, "Checksum mismatch"):
            runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
        self.assertEqual({path.relative_to(vq1): path.read_bytes()
                          for path in vq1.rglob("*") if path.is_file()}, before)
        self.assertEqual(list(vq1.parent.iterdir()), [vq1])

    def test_matching_payload_hashes_install(self):
        parts = self.archive("vq2")
        hashes = {path: hashlib.sha256(f"vq2: {path}".encode()).hexdigest()
                  for path in REQUIRED[1:]}
        sim = runtime.install(self.base, "vq2", parts, Path("."), REQUIRED, payload_sha256=hashes)
        self.assertTrue((sim / ".installed").is_file())
        for path, expected in hashes.items():
            self.assertEqual(hashlib.sha256((sim / path).read_bytes()).hexdigest(), expected)

    def test_wrong_executable_build_preserves_vq1_and_leaves_no_install(self):
        _, vq1 = self.cached_vq1()
        before = {path.relative_to(vq1): path.read_bytes()
                  for path in vq1.rglob("*") if path.is_file()}
        parts = self.archive("vq2")
        with self.assertRaisesRegex(SystemExit, "wrong .*Shipping.exe build"):
            runtime.install(self.base, "vq2", parts, Path("."), REQUIRED,
                            payload_sha256={REQUIRED[1]: "0" * 64})
        self.assertFalse((self.base / ".runtime/vq2").exists())
        self.assertEqual(list(vq1.parent.iterdir()), [vq1])
        self.assertEqual({path.relative_to(vq1): path.read_bytes()
                          for path in vq1.rglob("*") if path.is_file()}, before)

    def test_legacy_vq1_marker_reuses_install_without_archives(self):
        parts, sim = self.cached_vq1()
        self.assertFalse(any((self.base / name).exists() for _, name in parts))
        with patch.object(runtime.tarfile, "open", side_effect=AssertionError("archive accessed")):
            with patch.object(runtime.zipfile, "ZipFile", side_effect=AssertionError("archive accessed")):
                self.assertEqual(runtime.install(self.base, "vq1", parts, extract_vq1.ARCHIVE_ROOT,
                                                 extract_vq1.REQUIRED), sim)

    def test_independent_installations_and_configuration(self):
        installed = {}
        for name in ("vq1", "vq2"):
            source = self.base / f"{name}.ini"
            source.write_text(f"configuration for {name}")
            root = Path("nested") if name == "vq1" else Path(".")
            parts = self.archive(name, root)
            config = {source.name: Path("settings/config.ini")}
            sim = runtime.install(self.base, name, parts, root, REQUIRED, config)
            (self.base / parts[0][1]).unlink()
            source.write_text(f"updated {name}")
            self.assertEqual(runtime.install(self.base, name, parts, root, REQUIRED, config), sim)
            installed[name] = sim
        for name, sim in installed.items():
            self.assertEqual((sim / "settings/config.ini").read_text(), f"updated {name}")
            self.assertEqual((sim / REQUIRED[0]).read_text(), f"{name}: {REQUIRED[0]}")
        self.assertEqual(set((self.base / ".runtime").iterdir()), set(installed.values()))

    def test_interrupted_zip_cleans_staging(self):
        parts = self.archive("vq2")

        def interrupted(destination):
            (destination / "partial").write_bytes(b"incomplete")
            raise KeyboardInterrupt()

        with patch.object(zipfile.ZipFile, "extractall", side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):
                runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
        self.assertEqual(list((self.base / ".runtime").iterdir()), [])

    def test_failed_recovery_preparation_preserves_existing_install(self):
        sim = self.base / ".runtime/vq2"
        sim.mkdir(parents=True)
        (sim / "notes.txt").write_bytes(b"user data")
        parts = self.archive("vq2")
        archive = self.base / parts[0][1]
        original = archive.read_bytes()
        archive.write_bytes(b"broken download")
        with self.assertRaisesRegex(SystemExit, "Checksum mismatch"):
            runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
        self.assertEqual(list(sim.iterdir()), [sim / "notes.txt"])
        self.assertEqual((sim / "notes.txt").read_bytes(), b"user data")
        self.assertEqual(list(sim.parent.iterdir()), [sim])
        archive.write_bytes(original)
        with patch.object(runtime, "configure", side_effect=OSError("configuration failure")):
            with self.assertRaisesRegex(OSError, "configuration failure"):
                runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
        self.assertEqual(list(sim.iterdir()), [sim / "notes.txt"])
        self.assertEqual((sim / "notes.txt").read_bytes(), b"user data")
        self.assertEqual(list(sim.parent.iterdir()), [sim])

    def test_failed_replacement_rename_restores_previous_install(self):
        sim = self.base / ".runtime/vq2"
        sim.mkdir(parents=True)
        (sim / "notes.txt").write_bytes(b"user data")
        parts = self.archive("vq2")
        rename = Path.rename
        for error in (OSError("rename failed"), KeyboardInterrupt()):
            calls = []

            def replace(source, destination):
                calls.append((source, destination))
                if source.name.startswith("vq2-install-"):
                    raise error
                return rename(source, destination)

            with self.subTest(error=type(error).__name__):
                with patch.object(Path, "rename", replace):
                    with self.assertRaises(type(error)):
                        runtime.install(self.base, "vq2", parts, Path("."), REQUIRED)
                self.assertEqual(len(calls), 3)
                self.assertEqual(calls[0][0], sim)
                self.assertEqual(calls[-1], (calls[0][1], sim))
                self.assertEqual(list(sim.iterdir()), [sim / "notes.txt"])
                self.assertEqual((sim / "notes.txt").read_bytes(), b"user data")
                self.assertEqual(list(sim.parent.iterdir()), [sim])

    def test_vq2_recovery_preserves_vq1_and_old_vq2(self):
        _, vq1 = self.cached_vq1()
        before = {path.relative_to(vq1): path.read_bytes()
                  for path in vq1.rglob("*") if path.is_file()}
        sim = self.base / ".runtime/vq2"
        sim.mkdir()
        (sim / ".installed").write_bytes(b"\xff\x00invalid marker")
        (sim / "notes.txt").write_bytes(b"user data")
        parts = self.archive("vq2")
        self.assertEqual(runtime.install(self.base, "vq2", parts, Path("."), REQUIRED), sim)
        self.assertEqual((sim / REQUIRED[0]).read_text(), f"vq2: {REQUIRED[0]}")
        backups = list(sim.parent.glob("vq2-backup-*/vq2"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / ".installed").read_bytes(), b"\xff\x00invalid marker")
        self.assertEqual((backups[0] / "notes.txt").read_bytes(), b"user data")
        self.assertEqual({path.relative_to(vq1): path.read_bytes()
                          for path in vq1.rglob("*") if path.is_file()}, before)

    def test_vq1_wrapper_runs_directly_outside_repository(self):
        parts, sim = self.cached_vq1()
        for module in (extract_vq1, runtime):
            shutil.copyfile(module.__file__, self.base / Path(module.__file__).name)
        (self.base / "SHA256SUMS").write_text("\n".join(f"{digest}  {name}" for digest, name in parts))
        (self.base / "vq1").mkdir()
        for name in extract_vq1.CONFIG:
            (self.base / "vq1" / name).write_text(f"configuration {name}")
        result = subprocess.run([sys.executable, str(self.base / "extract_vq1.py")],
                                cwd=self.base, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        for name, relative in extract_vq1.CONFIG.items():
            self.assertEqual((sim / relative).read_text(), f"configuration {name}")


if __name__ == "__main__":
    unittest.main()
