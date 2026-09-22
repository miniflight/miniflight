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
LAUNCHER = Path(__file__).resolve().parents[1] / "target/aigp/run_vq1.sh"


@unittest.skipUnless(ZSH, "zsh is required")
class VQ1LauncherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name).resolve() / "sim setup"
        self.base.mkdir()
        self.events = self.base / "events.jsonl"
        helper = self.base / "stub.py"
        helper.write_text('''import json
import os
from pathlib import Path
import signal
import sys
import time

base = Path(os.environ["VQ1_TEST_BASE"])
kind = sys.argv[1]
with (base / "events.jsonl").open("a") as log:
    log.write(json.dumps({"kind": kind, "args": sys.argv[2:],
                          "cwd": os.getcwd(), "prefix": os.getenv("WINEPREFIX"),
                          "overrides": os.getenv("WINEDLLOVERRIDES"),
                          "pid": os.getpid()}) + "\\n")
if kind == "prepare":
    code = int(os.getenv("VQ1_TEST_PREPARE_EXIT", "0"))
    if not code:
        binary = base / ".runtime/vq1/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"
        binary.parent.mkdir(parents=True)
        binary.touch()
    sys.exit(code)
if kind == "wine":
    if os.getenv("VQ1_TEST_HOLD"):
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        (base / "wine.pid").write_text(str(os.getpid()))
        (base / "ready").touch()
        while True:
            time.sleep(1)
    sys.exit(int(os.getenv("VQ1_TEST_WINE_EXIT", "0")))
if kind == "wineserver" and (base / "wine.pid").exists():
    if os.environ["WINEPREFIX"] != str(base / ".runtime/vq1-wine"):
        sys.exit(9)
    try:
        os.kill(int((base / "wine.pid").read_text()), signal.SIGTERM)
    except ProcessLookupError:
        pass
''')
        self.stubs = {}
        for kind in ("prepare", "wine", "wineserver"):
            stub = self.base / kind
            stub.write_text(f"#!/bin/sh\nexec {shlex.quote(sys.executable)} "
                            f"{shlex.quote(str(helper))} {kind} \"$@\"\n")
            stub.chmod(0o755)
            self.stubs[kind] = stub
        source = LAUNCHER.read_text()
        for name, kind in (("WINE", "wine"), ("WINESERVER", "wineserver")):
            original = next(line for line in source.splitlines()
                            if line.startswith(f"readonly {name}="))
            source = source.replace(original, f"readonly {name}={shlex.quote(str(self.stubs[kind]))}")
        self.launcher = self.base / "run_vq1.sh"
        self.launcher.write_text(source)

    def launch(self, *args, **options):
        environment = os.environ.copy()
        environment.update(PYTHON=str(self.stubs["prepare"]), VQ1_TEST_BASE=str(self.base))
        environment.update(options)
        process = subprocess.Popen([ZSH, str(self.launcher), *args], env=environment,
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

    def test_prepare_failure_does_not_start_wine(self):
        code, output = self.finish(self.launch(VQ1_TEST_PREPARE_EXIT="7"))
        self.assertEqual(code, 7, output)
        events = self.read_events()
        self.assertEqual([event["kind"] for event in events], ["prepare"])
        self.assertEqual(events[0]["args"], [str(self.base / "extract_vq1.py")])

    def test_success_forwards_arguments_and_uses_local_runtime(self):
        code, output = self.finish(self.launch("-test", "value with spaces"))
        self.assertEqual(code, 0, output)
        events = self.read_events()
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

    def test_child_failure_is_preserved(self):
        code, output = self.finish(self.launch(VQ1_TEST_WINE_EXIT="23"))
        self.assertEqual(code, 23, output)
        self.assertEqual(self.read_events()[-1]["kind"], "wineserver")

    def test_sigint_exits_130_and_cleans_its_prefix(self):
        process = self.launch(VQ1_TEST_HOLD="1")
        deadline = time.monotonic() + 10
        while not (self.base / "ready").exists():
            if process.poll() is not None or time.monotonic() >= deadline:
                self.fail("Wine stub did not start")
            time.sleep(0.01)
        process.send_signal(signal.SIGINT)
        code, output = self.finish(process)
        self.assertEqual(code, 130, output)
        events = self.read_events()
        self.assertEqual([event["kind"] for event in events],
                         ["prepare", "wineserver", "wine", "wineserver"])
        self.assertEqual(events[-1]["args"], ["-k"])
        self.assertEqual(events[-1]["prefix"], str(self.base / ".runtime/vq1-wine"))


if __name__ == "__main__":
    unittest.main()
