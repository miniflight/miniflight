import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest


ZSH = shutil.which("zsh")
LAUNCHER = Path(__file__).resolve().parents[1] / "target/aigp/_runtime/run_vq1.sh"
WINE_HELPER = LAUNCHER.with_name("wine.sh")
PYTHON_HELPER = LAUNCHER.with_name("python.sh")


@unittest.skipUnless(ZSH, "zsh is required")
class VQ1LauncherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name).resolve() / "repo with spaces"
        self.base = self.repo / "target/aigp"
        self.base.mkdir(parents=True)
        (self.base / "_runtime").mkdir()
        self.python = self.base / ".runtime/client-venv/bin/python"
        self.events = self.base / "events.jsonl"
        helper = self.base / "stub.py"
        helper.write_text('''import json
import os
from pathlib import Path
import shutil
import signal
import sys
import time

base = Path(os.environ["VQ1_TEST_BASE"])
kind = sys.argv[1]
if kind == "prepare" and sys.argv[2:3] == ["-c"]:
    sys.exit(int(os.getenv("VQ1_TEST_PYTHON_VERSION_EXIT", "0")))
with (base / "events.jsonl").open("a") as log:
    log.write(json.dumps({"kind": kind, "args": sys.argv[2:],
                          "cwd": os.getcwd(), "prefix": os.getenv("WINEPREFIX"),
                          "overrides": os.getenv("WINEDLLOVERRIDES"),
                          "uv_cache": os.getenv("UV_CACHE_DIR"),
                          "uv_python": os.getenv("UV_PYTHON_INSTALL_DIR"),
                          "pid": os.getpid()}) + "\\n")
if kind == "uv":
    args = sys.argv[2:]
    if args[0] == "venv":
        code = int(os.getenv("VQ1_TEST_UV_VENV_EXIT", "0"))
        if not code:
            interpreter = Path(args[-1]) / "bin/python"
            interpreter.parent.mkdir(parents=True)
            shutil.copyfile(base / "prepare", interpreter)
            interpreter.chmod(0o755)
        sys.exit(code)
    if args[:2] == ["pip", "install"]:
        sys.exit(int(os.getenv("VQ1_TEST_UV_PIP_EXIT", "0")))
    sys.exit(99)
if kind == "prepare":
    code = int(os.getenv("VQ1_TEST_PREPARE_EXIT", "0"))
    if not code:
        binary = base / ".runtime/vq1/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"
        binary.parent.mkdir(parents=True, exist_ok=True)
        binary.touch()
    sys.exit(code)
if kind == "wine":
    if os.getenv("VQ1_TEST_HOLD"):
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        prefix = Path(os.environ["WINEPREFIX"])
        (prefix / "wine.pid").write_text(str(os.getpid()))
        (prefix / "ready").touch()
        while True:
            time.sleep(1)
    sys.exit(int(os.getenv("VQ1_TEST_WINE_EXIT", "0")))
if kind == "wineserver":
    pid_file = Path(os.environ["WINEPREFIX"]) / "wine.pid"
    if pid_file.exists():
        try:
            os.kill(int(pid_file.read_text()), signal.SIGTERM)
        except ProcessLookupError:
            pass
''')
        self.stubs = {}
        for kind in ("uv", "prepare", "wine", "wineserver"):
            stub = self.base / kind
            stub.write_text(f"#!/bin/sh\nexec {shlex.quote(sys.executable)} "
                            f"{shlex.quote(str(helper))} {kind} \"$@\"\n")
            stub.chmod(0o755)
            self.stubs[kind] = stub
        source = WINE_HELPER.read_text()
        for name, kind in (("WINE", "wine"), ("WINESERVER", "wineserver")):
            original = next(line for line in source.splitlines()
                            if line.startswith(f"readonly {name}="))
            source = source.replace(original, f"readonly {name}={shlex.quote(str(self.stubs[kind]))}")
        (self.base / "_runtime/wine.sh").write_text(source)
        (self.base / "_runtime/python.sh").write_text(PYTHON_HELPER.read_text())
        self.launcher = self.base / "_runtime/run_vq1.sh"
        self.launcher.write_text(LAUNCHER.read_text())

    def launch(self, *args, launcher=None, **options):
        environment = os.environ.copy()
        environment.update(PATH=str(self.base) + os.pathsep + environment.get("PATH", ""),
                           VQ1_TEST_BASE=str(self.base))
        environment.update(options)
        process = subprocess.Popen([ZSH, str(launcher or self.launcher), *args], env=environment,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True)
        self.addCleanup(self.cleanup_process, process)
        return process

    def cleanup_process(self, process):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.communicate(timeout=5)

    def finish(self, process):
        try:
            output = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            output = process.communicate(timeout=5)
            self.fail(f"launcher timed out: {output}")
        return process.returncode, output

    def read_events(self):
        return [json.loads(line) for line in self.events.read_text().splitlines()]

    def runtime_events(self):
        return [event for event in self.read_events() if event["kind"] != "uv"]

    def wait_for_wine(self, process, prefix):
        deadline = time.monotonic() + 10
        while not (prefix / "ready").exists():
            if process.poll() is not None or time.monotonic() >= deadline:
                self.fail("Wine stub did not start")
            time.sleep(0.01)

    def test_prepare_failure_does_not_start_wine(self):
        code, output = self.finish(self.launch(VQ1_TEST_PREPARE_EXIT="7"))
        self.assertEqual(code, 7, output)
        events = self.runtime_events()
        self.assertEqual([event["kind"] for event in events], ["prepare"])
        self.assertEqual(events[0]["args"], [str(self.base / "_runtime/vq1.py")])

    def test_existing_runtime_is_reused_in_place(self):
        runtime = self.base / ".runtime"
        interpreter = runtime / "client-venv/bin/python"
        interpreter.parent.mkdir(parents=True)
        shutil.copyfile(self.stubs["prepare"], interpreter)
        interpreter.chmod(0o755)
        (runtime / "keep.txt").write_text("existing runtime")
        code, output = self.finish(self.launch())
        self.assertEqual(code, 0, output)
        self.assertFalse(runtime.is_symlink())
        self.assertEqual((runtime / "keep.txt").read_text(), "existing runtime")
        self.assertFalse(any(event["kind"] == "uv" and event["args"][0] == "venv"
                             for event in self.read_events()))

    def test_success_forwards_arguments_and_uses_local_runtime(self):
        code, output = self.finish(self.launch("-test", "value with spaces"))
        self.assertEqual(code, 0, output)
        events = self.runtime_events()
        self.assertEqual([event["kind"] for event in events],
                         ["prepare", "wineserver", "wine", "wineserver"])
        wine = events[2]
        sim = self.base / ".runtime/vq1"
        self.assertEqual(wine["cwd"], str(sim))
        self.assertEqual(wine["args"], [
            str(sim / "FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"),
            "/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase",
            "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-NoSplash",
            "-test", "value with spaces",
        ])
        self.assertEqual(wine["overrides"], "dwmapi=n,b;winegstreamer=")
        for event in events[1:]:
            self.assertEqual(event["prefix"], str(self.base / ".runtime/vq1-wine"))
        for event in (events[1], events[3]):
            self.assertEqual(event["args"], ["-k"])

        bootstrap = [event for event in self.read_events() if event["kind"] == "uv"]
        self.assertEqual([event["args"] for event in bootstrap], [
            ["venv", "--python", "3.11", str(self.python.parent.parent)],
            ["pip", "install", "--python", str(self.python), "--editable", str(self.repo) + "[aigp]"],
        ])
        for event in bootstrap:
            self.assertEqual(event["uv_cache"], str(self.base / ".runtime/uv-cache"))
            self.assertEqual(event["uv_python"], str(self.base / ".runtime/uv-python"))
        self.assertEqual([event["kind"] for event in self.read_events()],
                         ["uv", "uv", "prepare", "wineserver", "wine", "wineserver"])

    def test_child_failure_is_preserved(self):
        code, output = self.finish(self.launch(VQ1_TEST_WINE_EXIT="23"))
        self.assertEqual(code, 23, output)
        self.assertEqual(self.read_events()[-1]["kind"], "wineserver")

    def test_sigint_exits_130_and_cleans_its_prefix(self):
        process = self.launch(VQ1_TEST_HOLD="1")
        self.wait_for_wine(process, self.base / ".runtime/vq1-wine")
        process.send_signal(signal.SIGINT)
        code, output = self.finish(process)
        self.assertEqual(code, 130, output)
        events = self.runtime_events()
        self.assertEqual([event["kind"] for event in events],
                         ["prepare", "wineserver", "wine", "wineserver"])
        self.assertEqual(events[-1]["args"], ["-k"])
        self.assertEqual(events[-1]["prefix"], str(self.base / ".runtime/vq1-wine"))

    def test_second_prefix_start_and_stop_leave_first_running(self):
        first_prefix = self.base / ".runtime/vq1-wine"
        first = self.launch(VQ1_TEST_HOLD="1")
        self.wait_for_wine(first, first_prefix)
        first_pid = int((first_prefix / "wine.pid").read_text())

        second_sim = self.base / "second sim"
        second_sim.mkdir()
        second_prefix = self.base / ".runtime/second-wine"
        launcher = self.base / "run_second.sh"
        launcher.write_text(
            "#!/bin/zsh\nset -eu\n"
            f"source {shlex.quote(str(self.base / '_runtime/wine.sh'))}\n"
            "check_wine\n"
            f"run_wine {shlex.quote(str(second_sim))} "
            f"{shlex.quote(str(second_prefix))} FlightSim.exe\n"
        )
        second = self.launch(launcher=launcher, VQ1_TEST_HOLD="1", WINEDLLOVERRIDES="")
        self.wait_for_wine(second, second_prefix)
        os.kill(first_pid, 0)
        self.assertIsNone(first.poll())

        second.send_signal(signal.SIGINT)
        code, output = self.finish(second)
        self.assertEqual(code, 130, output)
        os.kill(first_pid, 0)
        self.assertIsNone(first.poll())
        second_events = [event for event in self.read_events()
                         if event["prefix"] == str(second_prefix)]
        self.assertEqual([event["kind"] for event in second_events],
                         ["wineserver", "wine", "wineserver"])
        self.assertEqual(second_events[1]["cwd"], str(second_sim))
        self.assertEqual(second_events[1]["args"], ["FlightSim.exe"])
        self.assertEqual(second_events[1]["overrides"], "")

        first.send_signal(signal.SIGINT)
        code, output = self.finish(first)
        self.assertEqual(code, 130, output)

    def test_venv_bootstrap_failure_does_not_prepare_or_start_wine(self):
        code, output = self.finish(self.launch(VQ1_TEST_UV_VENV_EXIT="8"))
        self.assertEqual(code, 8, output)
        events = self.read_events()
        self.assertEqual([event["kind"] for event in events], ["uv"])
        self.assertEqual(events[0]["args"][0], "venv")
        self.assertFalse(self.python.exists())

    def test_dependency_install_failure_does_not_prepare_or_start_wine(self):
        code, output = self.finish(self.launch(VQ1_TEST_UV_PIP_EXIT="9"))
        self.assertEqual(code, 9, output)
        events = self.read_events()
        self.assertEqual([event["kind"] for event in events], ["uv", "uv"])
        self.assertEqual(events[-1]["args"][:2], ["pip", "install"])

    def test_cached_venv_is_reused(self):
        for _ in range(2):
            code, output = self.finish(self.launch())
            self.assertEqual(code, 0, output)
        bootstrap = [event["args"] for event in self.read_events() if event["kind"] == "uv"]
        self.assertEqual(sum(args[0] == "venv" for args in bootstrap), 1)
        self.assertEqual(sum(args[:2] == ["pip", "install"] for args in bootstrap), 2)
        prepares = [event for event in self.read_events() if event["kind"] == "prepare"]
        self.assertEqual(len(prepares), 2)

    def test_incompatible_cached_venv_is_preserved_and_recreated(self):
        self.python.parent.mkdir(parents=True)
        shutil.copyfile(self.stubs["prepare"], self.python)
        self.python.chmod(0o755)
        venv = self.python.parent.parent
        (venv / "old-environment").write_text("previous Python")

        code, output = self.finish(self.launch(VQ1_TEST_PYTHON_VERSION_EXIT="1"))
        self.assertEqual(code, 0, output)
        backups = list((self.base / ".runtime").glob("client-venv-backup-*/client-venv"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / "old-environment").read_text(), "previous Python")
        self.assertEqual((backups[0] / "bin/python").read_bytes(), self.stubs["prepare"].read_bytes())
        self.assertTrue(os.access(self.python, os.X_OK))
        self.assertFalse((venv / "old-environment").exists())
        self.assertEqual([event["kind"] for event in self.read_events()],
                         ["uv", "uv", "prepare", "wineserver", "wine", "wineserver"])

    def test_missing_cached_interpreter_is_preserved_and_recreated(self):
        self.python.parent.mkdir(parents=True)
        venv = self.python.parent.parent
        (venv / "old-environment").write_text("incomplete environment")

        code, output = self.finish(self.launch())
        self.assertEqual(code, 0, output)
        backups = list((self.base / ".runtime").glob("client-venv-backup-*/client-venv"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / "old-environment").read_text(), "incomplete environment")
        self.assertFalse((backups[0] / "bin/python").exists())
        self.assertTrue(os.access(self.python, os.X_OK))
        self.assertFalse((venv / "old-environment").exists())
        self.assertEqual([event["kind"] for event in self.read_events()],
                         ["uv", "uv", "prepare", "wineserver", "wine", "wineserver"])

    def test_missing_uv_fails_before_any_setup(self):
        code, output = self.finish(self.launch(PATH=str(self.base / "empty")))
        self.assertEqual(code, 1, output)
        self.assertIn("brew install uv", output[1])
        self.assertFalse(self.events.exists())
        self.assertFalse((self.base / ".runtime").exists())


if __name__ == "__main__":
    unittest.main()
