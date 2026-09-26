import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from target.aigp import install, runner


class WineDiscoveryTest(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.dict(os.environ, {}, clear=True))

    def resolve(self, platform, available):
        with patch.object(sys, "platform", platform), \
                patch.object(runner.shutil, "which", side_effect=available.get):
            return runner.wine_commands()

    def test_macos_uses_game_porting_toolkit(self):
        directory = "/Applications/Game Porting Toolkit.app/Contents/Resources/wine/bin/"
        available = {directory + name: directory + name for name in ("wine64", "wineserver")}
        self.assertEqual(self.resolve("darwin", available), (directory + "wine64", directory + "wineserver"))

    def test_linux_uses_path_and_matching_server(self):
        for name in ("wine64", "wine"):
            with self.subTest(name=name):
                available = {name: f"/usr/bin/{name}", "/usr/bin/wineserver": "/usr/bin/wineserver"}
                self.assertEqual(self.resolve("linux", available), (f"/usr/bin/{name}", "/usr/bin/wineserver"))

    def test_linux_distribution_can_put_server_elsewhere_on_path(self):
        self.assertEqual(self.resolve("linux", {"wine": "/usr/local/bin/wine", "wineserver": "/usr/bin/wineserver"}),
                         ("/usr/local/bin/wine", "/usr/bin/wineserver"))

    def test_custom_wine_never_silently_uses_an_unrelated_server(self):
        os.environ["WINE"] = "/custom/wine"
        available = {"/custom/wine": "/custom/wine", "wineserver": "/usr/bin/wineserver"}
        with self.assertRaisesRegex(FileNotFoundError, "matching executable"):
            self.resolve("linux", available)
        os.environ["WINESERVER"] = "/matching/server"
        available["/matching/server"] = "/matching/server"
        for platform in ("darwin", "linux"):
            self.assertEqual(self.resolve(platform, available), ("/custom/wine", "/matching/server"))

    def test_missing_wine_and_unsupported_platform_fail(self):
        with self.assertRaises(FileNotFoundError):
            self.resolve("linux", {})
        with self.assertRaises(RuntimeError):
            self.resolve("win32", {})

    def test_busy_udp_port_prevents_setup_or_prefix_cleanup(self):
        with patch.object(runner.socket, "socket") as socket, \
                patch.object(runner, "wine_commands") as wine, patch.object(runner, "prepare") as prepare:
            socket.return_value.__enter__.return_value.bind.side_effect = OSError("busy")
            with self.assertRaisesRegex(OSError, "UDP 14560"):
                with runner.launch("vq1.r1"):
                    self.fail("launched over an existing simulator")
            wine.assert_not_called()
            prepare.assert_not_called()


class LauncherTest(unittest.TestCase):
    """Real process/signal tests with a Wine stand-in, not a graphics compatibility test."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name).resolve() / "path with spaces"
        self.base.mkdir()
        (self.base / "archives").mkdir()
        (self.base / "archives/SHA256SUMS").write_text("\n".join(f"{'0' * 64}  {name}.tar.xz" for name in install.VERSIONS))
        for version, profile in install.VERSIONS.items():
            sim = self.base / ".runtime" / version
            for relative in install.REQUIRED:
                path = sim / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"test payload")
            (sim / ".installed").write_text(profile["legacy"])
            for name in install.configuration(version):
                source = self.base / "config" / version / name
                source.parent.mkdir(parents=True, exist_ok=True)
                source.write_text(f"test config {version} {name}")
        self.events = self.base / "events.jsonl"
        helper = self.base / "stub.py"
        helper.write_text('''import json, os, signal, sys, time
from pathlib import Path
base = Path(os.environ["AIGP_TEST_BASE"])
prefix = Path(os.environ["WINEPREFIX"])
kind = sys.argv[1]
with (base / "events.jsonl").open("a") as log:
    log.write(json.dumps({"kind": kind, "args": sys.argv[2:], "cwd": os.getcwd(),
                          "prefix": str(prefix), "mode": os.getenv("MINIFLIGHT_VQ2_MODE"),
                          "overrides": os.getenv("WINEDLLOVERRIDES"),
                          "dxgi": os.getenv("WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER"),
                          "pid": os.getpid()}) + "\\n")
if kind == "wine":
    if os.getenv("AIGP_TEST_HOLD"):
        (prefix / "wine.pid").write_text(str(os.getpid()))
        while True:
            time.sleep(.1)
    sys.exit(int(os.getenv("AIGP_TEST_EXIT", "0")))
if kind == "wineserver":
    if os.getenv("AIGP_TEST_SERVER_EXIT"):
        sys.exit(int(os.environ["AIGP_TEST_SERVER_EXIT"]))
    pid = prefix / "wine.pid"
    if pid.exists():
        try:
            os.kill(int(pid.read_text()), signal.SIGTERM)
        except ProcessLookupError:
            pass
''')
        for kind in ("wine", "wineserver"):
            stub = self.base / kind
            stub.write_text(f'#!/bin/sh\nexec {shlex.quote(sys.executable)} {shlex.quote(str(helper))} {kind} "$@"\n')
            stub.chmod(0o755)
        self.env = dict(os.environ, WINE=str(self.base / "wine"), WINESERVER=str(self.base / "wineserver"),
                        AIGP_TEST_BASE=str(self.base))
        for name in ("MINIFLIGHT_VQ2_MODE", "WINE_DO_NOT_CREATE_DXGI_DEVICE_MANAGER"):
            self.env.pop(name, None)

    def launch(self, target="vq1.r1", *args, platform=sys.platform, **env):
        code = ("from target.aigp import runner; import sys; from pathlib import Path; "
                "runner.BASE = Path(sys.argv[1]); runner.sys.platform = sys.argv[2]; "
                "runner._check_simulator_ports = lambda: None; sys.exit(runner.main(sys.argv[3:]))")
        process = subprocess.Popen([sys.executable, "-c", code, str(self.base), platform, target, *args],
                                   cwd=Path(runner.__file__).resolve().parents[2], env=dict(self.env, **env),
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
        self.addCleanup(self.cleanup, process)
        return process

    def cleanup(self, process):
        if process.poll() is None:
            process.terminate()
        try:
            process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5)
        # Only PIDs recorded by this test's own Wine stand-ins.
        for path in (self.base / ".runtime").glob("*-wine/wine.pid"):
            try:
                os.kill(int(path.read_text()), signal.SIGTERM)
            except ProcessLookupError:
                pass

    def finish(self, process, expected=0):
        output = process.communicate(timeout=15)
        self.assertEqual(process.returncode, expected, output)
        return output

    def read_events(self):
        return [json.loads(line) for line in self.events.read_text().splitlines()] if self.events.exists() else []

    def wait_for_wine(self, process, version):
        pid = self.base / ".runtime" / f"{version}-wine/wine.pid"
        deadline = time.monotonic() + 10
        while not pid.exists():
            if process.poll() is not None or time.monotonic() >= deadline:
                self.fail(f"Wine stand-in failed to start: {process.communicate(timeout=1)}")
            time.sleep(.01)
        return int(pid.read_text())

    def test_all_targets_on_both_launch_paths(self):
        for platform in ("darwin", "linux"):
            for target, (version, mode, level) in runner.TARGETS.items():
                with self.subTest(platform=platform, target=target):
                    before = len(self.read_events())
                    self.finish(self.launch(target, "-test", "value with spaces", platform=platform))
                    events = self.read_events()[before:]
                    self.assertEqual([event["kind"] for event in events], ["wineserver", "wine", "wineserver"])
                    sim = self.base / ".runtime" / version
                    wine = events[1]
                    self.assertEqual(wine["cwd"], str(sim))
                    self.assertEqual(wine["args"], [str(sim / install.SHIPPING),
                        f"/Game/levelsMaster/{level}?game=/Script/DCGame.GameModeRaceBase",
                        "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-NoSplash", "-test", "value with spaces"])
                    self.assertEqual(wine["mode"], mode if version == "vq2" else None)
                    self.assertEqual(wine["overrides"], "dwmapi=n,b;winegstreamer=")
                    self.assertEqual(wine["dxgi"], "1" if platform == "darwin" else None)
                    for event in events:
                        self.assertEqual(event["prefix"], str(self.base / ".runtime" / f"{version}-wine"))
                    for event in (events[0], events[2]):
                        self.assertEqual(event["args"], ["-k"])

    def test_existing_installs_and_old_python_environment_are_preserved(self):
        old_python = self.base / ".runtime/client-venv/bin/python"
        old_python.parent.mkdir(parents=True)
        old_python.write_text("existing python environment")
        self.finish(self.launch())
        self.assertEqual(old_python.read_text(), "existing python environment")
        self.assertEqual((self.base / ".runtime/vq1" / install.SHIPPING).read_bytes(), b"test payload")

    def test_prepare_failure_never_starts_wine(self):
        (self.base / "archives/SHA256SUMS").unlink()
        self.finish(self.launch(), expected=1)
        self.assertEqual(self.read_events(), [])

    def test_server_failure_never_starts_wine(self):
        self.finish(self.launch(AIGP_TEST_SERVER_EXIT="7"), expected=1)
        self.assertTrue(all(event["kind"] == "wineserver" for event in self.read_events()))

    def test_no_running_server_is_not_a_startup_failure(self):
        self.finish(self.launch(AIGP_TEST_SERVER_EXIT="1"))
        self.assertEqual([event["kind"] for event in self.read_events()], ["wineserver", "wine", "wineserver"])

    def test_child_failure_is_preserved(self):
        self.finish(self.launch("vq2.r2", AIGP_TEST_EXIT="23"), expected=23)
        self.assertEqual(self.read_events()[-1]["kind"], "wineserver")

    def test_signals_clean_the_owned_prefix(self):
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signum=signum):
                pid_path = self.base / ".runtime/vq2-wine/wine.pid"
                pid_path.unlink(missing_ok=True)
                process = self.launch("vq2.r2", AIGP_TEST_HOLD="1")
                pid = self.wait_for_wine(process, "vq2")
                process.send_signal(signum)
                self.finish(process, expected=128 + signum)
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)
                self.assertEqual(self.read_events()[-1]["kind"], "wineserver")

    def test_same_prefix_is_refused_without_stopping_its_owner(self):
        first = self.launch(AIGP_TEST_HOLD="1")
        pid = self.wait_for_wine(first, "vq1")
        before = self.read_events()
        output = self.finish(self.launch(), expected=1)
        self.assertIn("already running", output[1])
        self.assertEqual(self.read_events(), before)
        os.kill(pid, 0)
        self.assertIsNone(first.poll())

    def test_stopping_another_prefix_does_not_kill_the_first(self):
        first = self.launch(AIGP_TEST_HOLD="1")
        pid = self.wait_for_wine(first, "vq1")
        second = self.launch("vq2.r2", AIGP_TEST_HOLD="1")
        self.wait_for_wine(second, "vq2")
        second.send_signal(signal.SIGINT)
        self.finish(second, expected=130)
        os.kill(pid, 0)
        self.assertIsNone(first.poll())


if __name__ == "__main__":
    unittest.main()
