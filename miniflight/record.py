"""Append-only raw flight record."""

from __future__ import annotations

import json
import time
from pathlib import Path

from miniflight.core.machine import MachineProfile


def read_flight_record(path: Path):
    """Yield one validated flight record in sequence order."""
    previous_time = -1
    record_count = 0
    with path.open(encoding="utf-8") as record_file:
        for line_number, line in enumerate(record_file, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON at line {line_number}") from error
            if not isinstance(record, dict):
                raise ValueError(f"invalid record at line {line_number}")
            if record.get("sequence") != line_number - 1:
                raise ValueError(f"invalid sequence at line {line_number}")
            record_time = record.get("monotonic_time_ns")
            if not isinstance(record_time, int) or record_time < previous_time:
                raise ValueError(f"invalid monotonic time at line {line_number}")
            if line_number == 1 and (
                record.get("type") != "header"
                or record.get("format") != "miniflight-flight-record"
                or record.get("version") != 1
            ):
                raise ValueError("invalid flight record header")
            previous_time = record_time
            record_count += 1
            yield record
    if record_count == 0:
        raise ValueError("empty flight record")


class FlightRecord:
    """Write ordered transport events without decoding them."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = path.open("w", encoding="utf-8")
        self._sequence = 0
        self._write(
            {
                "type": "header",
                "format": "miniflight-flight-record",
                "version": 1,
                "wall_time_ns": time.time_ns(),
            }
        )

    def request(self, frame: bytes) -> None:
        self._write({"type": "msp_request", "frame_hex": frame.hex()})

    def response(self, frame: bytes) -> None:
        self._write({"type": "msp_response", "frame_hex": frame.hex()})

    def link(self, state: str, device: str) -> None:
        self._write({"type": "link", "state": state, "device": device})

    def machine(self, device: str, profile: MachineProfile, poll_plan: dict) -> None:
        self._write(
            {
                "type": "machine",
                "device": device,
                "controller": profile.controller,
                "firmware": profile.firmware,
                "api_version": profile.api_version,
                "board": profile.board,
                "sensors": sorted(profile.sensors),
                "motor_count": profile.motor_count,
                "poll_plan": poll_plan,
            }
        )

    def close(self) -> None:
        if not self._file.closed:
            self._write({"type": "closed"})
            self._file.close()

    def _write(self, record: dict) -> None:
        record["sequence"] = self._sequence
        record["monotonic_time_ns"] = time.monotonic_ns()
        self._sequence += 1
        self._file.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
        self._file.flush()
