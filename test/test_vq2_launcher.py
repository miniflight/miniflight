from pathlib import Path
import signal
import unittest

from test import test_vq1_launcher as shared


LAUNCHER = Path(__file__).resolve().parents[1] / "target/aigp/_runtime/run_vq2.sh"


@unittest.skipUnless(shared.ZSH, "zsh is required")
class VQ2LauncherTest(unittest.TestCase):
    # Exercise the same dependency/process fixtures as the working VQ1 runner.
    launch = shared.VQ1LauncherTest.launch
    cleanup_process = shared.VQ1LauncherTest.cleanup_process
    finish = shared.VQ1LauncherTest.finish
    read_events = shared.VQ1LauncherTest.read_events
    runtime_events = shared.VQ1LauncherTest.runtime_events
    wait_for_wine = shared.VQ1LauncherTest.wait_for_wine
    test_child_failure_is_preserved = shared.VQ1LauncherTest.test_child_failure_is_preserved
    test_cached_venv_is_reused = shared.VQ1LauncherTest.test_cached_venv_is_reused
    test_missing_uv_fails_before_any_setup = shared.VQ1LauncherTest.test_missing_uv_fails_before_any_setup
    test_venv_bootstrap_failure_does_not_prepare_or_start_wine = (
        shared.VQ1LauncherTest.test_venv_bootstrap_failure_does_not_prepare_or_start_wine)
    test_dependency_install_failure_does_not_prepare_or_start_wine = (
        shared.VQ1LauncherTest.test_dependency_install_failure_does_not_prepare_or_start_wine)

    def setUp(self):
        shared.VQ1LauncherTest.setUp(self)
        helper = self.base / "stub.py"
        source = helper.read_text().replace(".runtime/vq1/", ".runtime/vq2/")
        source = source.replace('"pid": os.getpid()',
                                '"mode": os.getenv("MINIFLIGHT_VQ2_MODE"), "pid": os.getpid()')
        helper.write_text(source)
        self.launcher = self.base / "_runtime/run_vq2.sh"
        self.launcher.write_text(LAUNCHER.read_text())

    def assert_mode(self, mode, master, *selection):
        code, output = self.finish(self.launch(*selection, "-test", "value with spaces"))
        self.assertEqual(code, 0, output)
        events = self.runtime_events()
        self.assertEqual([event["kind"] for event in events],
                         ["prepare", "wineserver", "wine", "wineserver"])
        self.assertEqual(events[0]["args"], [str(self.base / "_runtime/vq2.py")])
        sim = self.base / ".runtime/vq2"
        wine = events[2]
        self.assertEqual(wine["cwd"], str(sim))
        self.assertEqual(wine["args"], [
            str(sim / "FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe"),
            f"/Game/levelsMaster/{master}?game=/Script/DCGame.GameModeRaceBase",
            "-windowed", "-ResX=1280", "-ResY=720", "-nosound", "-NoSplash",
            "-test", "value with spaces",
        ])
        self.assertEqual(wine["mode"], mode)
        self.assertEqual(wine["overrides"], "dwmapi=n,b;winegstreamer=")
        for event in events[1:]:
            self.assertEqual(event["prefix"], str(self.base / ".runtime/vq2-wine"))
        uv = [event for event in self.read_events() if event["kind"] == "uv"]
        self.assertEqual(uv[0]["args"], ["venv", "--python", "3.11", str(self.python.parent.parent)])
        self.assertEqual(uv[1]["args"], ["pip", "install", "--python", str(self.python),
                                        "--editable", str(self.repo) + "[aigp]"])

    def test_default_round_is_r1(self):
        self.assert_mode("r1", "MAP_anduril_master")

    def test_r1_profile(self):
        self.assert_mode("r1", "MAP_anduril_master", "--mode", "r1")

    def test_r2_profile(self):
        self.assert_mode("r2", "MAP_arsenal_master", "--mode", "r2")

    def test_invalid_or_missing_round_fails_before_setup(self):
        for args in (("--mode",), ("--mode", "r3")):
            with self.subTest(args=args):
                code, output = self.finish(self.launch(*args))
                self.assertEqual(code, 2, output)
                self.assertFalse(self.events.exists())

    def test_prepare_failure_does_not_start_wine(self):
        code, output = self.finish(self.launch(VQ1_TEST_PREPARE_EXIT="7"))
        self.assertEqual(code, 7, output)
        self.assertEqual([event["kind"] for event in self.runtime_events()], ["prepare"])

    def test_sigint_cleans_only_vq2_prefix(self):
        process = self.launch(VQ1_TEST_HOLD="1")
        self.wait_for_wine(process, self.base / ".runtime/vq2-wine")
        process.send_signal(signal.SIGINT)
        code, output = self.finish(process)
        self.assertEqual(code, 130, output)
        events = self.runtime_events()
        self.assertEqual([event["kind"] for event in events],
                         ["prepare", "wineserver", "wine", "wineserver"])
        self.assertEqual(events[-1]["prefix"], str(self.base / ".runtime/vq2-wine"))


if __name__ == "__main__":
    unittest.main()
