"""Append-only raw flight record."""

from __future__ import annotations

import json
import time
from pathlib import Path


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
