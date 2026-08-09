"""Regression tests for replayable flights."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from miniflight.__main__ import main
from miniflight.program import PositionNed
from miniflight.replay import _run_thread_gates_process, replay


FLIGHT = Path(__file__).with_name("data") / "vq1_six_gates.mflog"


def test_thread_gates_runs_in_a_fresh_process():
    worker_pid, commands = _run_thread_gates_process(
        ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)),
        [0, 1],
    )

    assert worker_pid != os.getpid()
    assert commands == [PositionNed((1.0, 2.0, 3.0)), PositionNed((4.0, 5.0, 6.0))]


def test_vq1_six_gate_flight_replays(capsys):
    summary = replay(FLIGHT)
    main(["replay", str(FLIGHT)])

    assert summary == {
        "target": "vq1",
        "program": "thread-gates",
        "gates": [0, 1, 2, 3, 4, 5, 6],
        "collisions": 0,
        "result": "pass",
    }
    assert capsys.readouterr().out == (
        "target vq1\n"
        "program thread-gates\n"
        "gates 0 1 2 3 4 5 6\n"
        "collisions 0\n"
        "result pass\n"
    )


def test_replay_command_is_self_contained():
    completed = subprocess.run(
        [sys.executable, "-m", "miniflight", "replay", str(FLIGHT)],
        cwd=Path(__file__).parents[1],
        check=True,
        capture_output=True,
        text=True,
    )

    assert completed.stdout.endswith("result pass\n")


def test_replay_does_not_depend_on_the_simulator(tmp_path):
    records = [json.loads(line) for line in FLIGHT.read_text().splitlines()]
    records[1]["name"] = "another-simulator"
    alternate = tmp_path / "alternate.mflog"
    alternate.write_text("".join(json.dumps(record) + "\n" for record in records))

    assert replay(alternate)["target"] == "another-simulator"


def test_replay_rejects_a_changed_program_command(tmp_path):
    records = [json.loads(line) for line in FLIGHT.read_text().splitlines()]
    records[4]["command"]["position_ned_m"][0] += 1.0
    changed = tmp_path / "changed.mflog"
    changed.write_text("".join(json.dumps(record) + "\n" for record in records))

    with pytest.raises(ValueError, match="command mismatch at gate 0"):
        replay(changed)
