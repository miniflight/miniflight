"""Replay recorded FlightProgram decisions in a fresh process."""

from __future__ import annotations

import multiprocessing
import os
from pathlib import Path

from miniflight.program import PositionNed
from miniflight.race import ThreadGatesProgram
from miniflight.record import read_flight_record


_PROCESS_TIMEOUT_S = 5.0


def _one(records: list[dict], record_type: str) -> dict:
    matches = [record for record in records if record.get("type") == record_type]
    if len(matches) != 1:
        raise ValueError(f"flight record requires one {record_type} record")
    return matches[0]


def _thread_gates_process(connection) -> None:
    """Run the recorded program behind a process boundary."""
    program = None
    try:
        centers = connection.recv()
        program = ThreadGatesProgram(centers)
        program.start()
        connection.send(("ready", os.getpid()))
        while True:
            active_gate = connection.recv()
            if active_gate is None:
                break
            program.select_gate(active_gate)
            command = program.step()
            connection.send(
                (
                    "command",
                    command.position_ned_m,
                    command.yaw_rad,
                )
            )
    finally:
        if program is not None:
            program.stop("replay complete")
        connection.close()


def _receive(connection):
    if not connection.poll(_PROCESS_TIMEOUT_S):
        raise RuntimeError("flight program process did not respond")
    return connection.recv()


def _run_thread_gates_process(
    centers: tuple[tuple[float, float, float], ...],
    active_gates: list[int],
) -> tuple[int, list[PositionNed]]:
    """Run ThreadGatesProgram in a fresh process."""
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=_thread_gates_process, args=(child,))
    process.start()
    child.close()
    commands = []
    worker_pid = -1
    try:
        parent.send(centers)
        ready = _receive(parent)
        if ready[0] != "ready":
            raise RuntimeError("flight program process did not become ready")
        worker_pid = ready[1]

        for active_gate in active_gates:
            parent.send(active_gate)
            response = _receive(parent)
            if response[0] != "command":
                raise RuntimeError("flight program process returned an invalid response")
            commands.append(PositionNed(response[1], response[2]))

        parent.send(None)
    finally:
        parent.close()
        process.join(_PROCESS_TIMEOUT_S)
        if process.is_alive():
            process.terminate()
            process.join()

    if process.exitcode != 0:
        raise RuntimeError(f"flight program process exited with {process.exitcode}")
    return worker_pid, commands


def replay(path: Path) -> dict:
    """Replay one supported program and return its verified result."""
    records = list(read_flight_record(path))
    target = _one(records, "target")
    track = _one(records, "track")
    program_record = _one(records, "program")
    result = _one(records, "result")

    if program_record.get("name") != "thread-gates":
        raise ValueError(f"unsupported program {program_record.get('name')}")

    try:
        centers = tuple(
            tuple(float(value) for value in center)
            for center in track["gate_centers_ned_m"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("invalid gate centers") from error

    commands = [record for record in records if record.get("type") == "command"]
    if len(commands) != len(centers):
        raise ValueError("flight record requires one command for each gate")

    active_gates = [record.get("active_gate") for record in commands]
    expected_gates = list(range(len(centers)))
    if active_gates != expected_gates:
        raise ValueError("active gate sequence does not increase")
    _, generated_commands = _run_thread_gates_process(centers, expected_gates)

    for expected_gate, (record, generated) in enumerate(
        zip(commands, generated_commands)
    ):
        recorded = record.get("command")
        if not isinstance(recorded, dict) or recorded.get("type") != "position_ned":
            raise ValueError(f"invalid command for gate {expected_gate}")
        expected = PositionNed(
            tuple(recorded.get("position_ned_m", ())),
            recorded.get("yaw_rad"),
        )
        if generated != expected:
            raise ValueError(f"command mismatch at gate {expected_gate}")

    gate_sequence = result.get("active_gate_sequence")
    expected_sequence = list(range(len(centers) + 1))
    if gate_sequence != expected_sequence:
        raise ValueError("result does not contain the complete gate sequence")
    if result.get("passed_gate_count") != len(centers):
        raise ValueError("result gate count does not match the track")

    return {
        "target": target.get("name"),
        "program": program_record["name"],
        "gates": gate_sequence,
        "collisions": result.get("collision_count"),
        "result": result.get("status"),
    }


def print_replay(path: Path) -> dict:
    """Replay one record and print its stable result summary."""
    summary = replay(path)
    print(f"target {summary['target']}")
    print(f"program {summary['program']}")
    print("gates " + " ".join(str(gate) for gate in summary["gates"]))
    print(f"collisions {summary['collisions']}")
    print(f"result {summary['result']}")
    return summary


__all__ = ["print_replay", "replay"]
