from contextlib import redirect_stdout
import hashlib
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from target.aigp import install as runtime


class VQ2PackageTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.sim = self.base / ".runtime/vq2"
        (self.base / "config/vq2").mkdir(parents=True)
        (self.base / "archives").mkdir()
        for name in runtime.configuration("vq2"):
            (self.base / "config/vq2" / name).write_text(f"configuration {name}")
        self.enterContext(redirect_stdout(io.StringIO()))

    def archive(self):
        data = io.BytesIO()
        with tarfile.open(fileobj=data, mode="w:gz") as archive:
            for path in runtime.REQUIRED:
                content = str(path).encode()
                member = tarfile.TarInfo(str(runtime.VERSIONS["vq2"]["root"] / path))
                member.size = len(content)
                archive.addfile(member, io.BytesIO(content))
        self.part = self.base / "archives/vq2-unlocked.tar.gz.part-aa"
        self.part.write_bytes(data.getvalue())
        self.parts = [[hashlib.sha256(data.getvalue()).hexdigest(), self.part.name]]
        (self.base / "archives/SHA256SUMS").write_text(
            f"{self.parts[0][0]}  {self.part.name}\n{'0' * 64}  vq1-unlocked.tar.gz.part-aa\n")
        hashes = {path: hashlib.sha256(str(path).encode()).hexdigest()
                  for path in runtime.VERSIONS["vq2"]["hashes"]}
        self.enterContext(patch.dict(runtime.VERSIONS["vq2"], hashes=hashes))

    def test_uses_shared_installer_with_vq2_metadata(self):
        self.archive()
        with patch.object(runtime, "install", return_value=self.sim) as install:
            self.assertEqual(runtime.prepare("vq2", self.base), self.sim)
        config = {Path("config/vq2") / name: path for name, path in runtime.configuration("vq2").items()}
        install.assert_called_once_with(self.base, "vq2", self.parts, runtime.VERSIONS["vq2"]["root"],
                                        runtime.REQUIRED, config, runtime.VERSIONS["vq2"]["hashes"],
                                        archive_dir=self.base / "archives",
                                        cache_versions=(runtime.VERSIONS["vq2"]["legacy"],))

    def test_install_reuse_and_configuration_refresh(self):
        self.archive()
        self.assertEqual(runtime.prepare("vq2", self.base), self.sim)
        for relative in runtime.REQUIRED:
            self.assertEqual((self.sim / relative).read_bytes(), str(relative).encode())
        for name in runtime.configuration("vq2"):
            (self.base / "config/vq2" / name).write_text(f"updated {name}")
        self.part.unlink()
        with patch.object(runtime.tarfile, "open", side_effect=AssertionError("re-extraction")):
            self.assertEqual(runtime.prepare("vq2", self.base), self.sim)
        for name, relative in runtime.configuration("vq2").items():
            self.assertEqual((self.sim / relative).read_text(), f"updated {name}")

    def test_bad_payload_preserves_vq1_and_does_not_install_vq2(self):
        self.archive()
        vq1 = self.base / ".runtime/vq1"
        vq1.mkdir(parents=True)
        (vq1 / "existing.txt").write_bytes(b"unchanged")
        with patch.dict(runtime.VERSIONS["vq2"], hashes={runtime.SHIPPING: "0" * 64}):
            with self.assertRaisesRegex(SystemExit, "wrong .*Shipping.exe build"):
                runtime.prepare("vq2", self.base)
        self.assertEqual((vq1 / "existing.txt").read_bytes(), b"unchanged")
        self.assertFalse(self.sim.exists())
        self.assertEqual(list(vq1.parent.iterdir()), [vq1])

    def test_direct_execution_reuses_installed_package(self):
        self.archive()
        runtime.prepare("vq2", self.base)
        shutil.copyfile(runtime.__file__, self.base / "install.py")
        self.part.unlink()
        result = subprocess.run([sys.executable, str(self.base / "install.py"), "vq2"],
                                cwd=self.base, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
