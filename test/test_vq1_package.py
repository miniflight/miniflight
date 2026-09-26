import hashlib
import io
from pathlib import Path
import tarfile
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from target.aigp import install as runtime


class VQ1PackageTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.sim = self.base / ".runtime/vq1"
        (self.base / "config/vq1").mkdir(parents=True)
        (self.base / "archives").mkdir()
        for name in runtime.configuration("vq1"):
            (self.base / "config/vq1" / name).write_text(f"source {name}")
        self.archive()
        self.enterContext(redirect_stdout(io.StringIO()))

    def archive(self, missing=None):
        compressed = io.BytesIO()
        with tarfile.open(fileobj=compressed, mode="w:gz") as archive:
            for path in runtime.REQUIRED:
                if path == missing:
                    continue
                data = str(path).encode()
                member = tarfile.TarInfo(str(runtime.VERSIONS["vq1"]["root"] / path))
                member.size = len(data)
                archive.addfile(member, io.BytesIO(data))
        data = compressed.getvalue()
        halves = (data[:len(data) // 2], data[len(data) // 2:])
        self.parts = []
        checksums = []
        for suffix, data in zip(("aa", "ab"), halves):
            path = self.base / "archives" / f"vq1-unlocked.tar.gz.part-{suffix}"
            path.write_bytes(data)
            self.parts.append(path)
            checksums.append(f"{hashlib.sha256(data).hexdigest()}  {path.name}")
        checksums.append(f"{'0' * 64}  VQ1-Technical-Specification-00.02.pdf")
        (self.base / "archives/SHA256SUMS").write_text("\n".join(checksums) + "\n")

    def assert_no_install(self):
        self.assertFalse(self.sim.exists())
        runtime = self.base / ".runtime"
        if runtime.exists():
            self.assertEqual(list(runtime.iterdir()), [])

    def test_install_and_reuse_refreshes_configuration(self):
        self.assertEqual(runtime.prepare("vq1", self.base), self.sim)
        self.assertTrue((self.sim / ".installed").is_file())
        for path in runtime.REQUIRED:
            self.assertEqual((self.sim / path).read_bytes(), str(path).encode())
        for name, path in runtime.configuration("vq1").items():
            self.assertEqual((self.sim / path).read_text(), f"source {name}")
            (self.base / "config/vq1" / name).write_text(f"updated {name}")
        for path in self.parts:
            path.unlink()
        with patch.object(tarfile, "open", side_effect=AssertionError("re-extraction")):
            self.assertEqual(runtime.prepare("vq1", self.base), self.sim)
        for name, path in runtime.configuration("vq1").items():
            self.assertEqual((self.sim / path).read_text(), f"updated {name}")
        self.assertEqual(list(self.sim.parent.iterdir()), [self.sim])

    def test_missing_part_leaves_no_install(self):
        self.parts[-1].unlink()
        with self.assertRaisesRegex(SystemExit, "Missing"):
            runtime.prepare("vq1", self.base)
        self.assert_no_install()

    def test_lfs_pointer_leaves_no_install(self):
        self.parts[0].write_text("version https://git-lfs.github.com/spec/v1\n"
                                 f"oid sha256:{'0' * 64}\nsize 100\n")
        with self.assertRaisesRegex(SystemExit, "Checksum mismatch"):
            runtime.prepare("vq1", self.base)
        self.assert_no_install()

    def test_corrupt_part_leaves_no_install(self):
        self.parts[-1].write_bytes(b"corrupt")
        with self.assertRaisesRegex(SystemExit, "Checksum mismatch"):
            runtime.prepare("vq1", self.base)
        self.assert_no_install()

    def test_extraction_failure_and_interrupt_clean_staging(self):
        for error in (RuntimeError("extraction failed"), KeyboardInterrupt()):
            def fail_extraction(destination, **kwargs):
                (destination / "partial").write_bytes(b"unfinished")
                raise error

            with self.subTest(error=type(error).__name__):
                with patch.object(tarfile.TarFile, "extractall", side_effect=fail_extraction):
                    with self.assertRaises(type(error)):
                        runtime.prepare("vq1", self.base)
                self.assert_no_install()

    def test_executable_only_install_is_recovered_and_preserved(self):
        executable = self.sim / runtime.REQUIRED[0]
        executable.parent.mkdir(parents=True)
        executable.write_bytes(b"partial")
        (self.sim / "notes.txt").write_text("keep my notes")
        self.assertEqual(runtime.prepare("vq1", self.base), self.sim)
        self.assertEqual(executable.read_bytes(), str(runtime.REQUIRED[0]).encode())
        self.assertTrue((self.sim / ".installed").is_file())
        backups = list(self.sim.parent.glob("vq1-backup-*/vq1"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / runtime.REQUIRED[0]).read_bytes(), b"partial")
        self.assertEqual((backups[0] / "notes.txt").read_text(), "keep my notes")
        self.assertFalse((backups[0] / ".installed").exists())

    def test_missing_required_asset_leaves_no_install(self):
        self.archive(missing=runtime.REQUIRED[-1])
        with self.assertRaisesRegex(SystemExit, "missing required simulator files"):
            runtime.prepare("vq1", self.base)
        self.assert_no_install()

    def test_stale_marker_is_recovered_and_preserved(self):
        runtime.prepare("vq1", self.base)
        marker = self.sim / ".installed"
        marker.write_text("old version")
        self.assertEqual(runtime.prepare("vq1", self.base), self.sim)
        self.assertNotEqual(marker.read_text(), "old version")
        backups = list(self.sim.parent.glob("vq1-backup-*/vq1"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / ".installed").read_text(), "old version")

    def test_pdf_is_not_required(self):
        self.assertFalse(list(self.base.glob("*.pdf")))
        self.assertEqual(runtime.prepare("vq1", self.base), self.sim)


if __name__ == "__main__":
    unittest.main()
