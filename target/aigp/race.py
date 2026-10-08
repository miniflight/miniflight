"""Read the UE4SS race observer; these observations are separate from MAVLink."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time


@dataclass(frozen=True)
class NativeRaceState:
    started: bool  # GameStateRaceBase.HasRaceStarted()
    valid: bool  # Local PlayerStateRaceBase.IsRaceValid()
    completed: bool  # Local PlayerStateRaceBase.bRaceCompleted
    time_seconds: float  # GameStateRaceBase.GetRaceTime(), not host elapsed time
    active_gate_index: int  # Local PlayerStateRaceBase.ActiveGateSequentialId
    finish_time_seconds: float  # Local PlayerStateRaceBase.GetCompletedRaceTotalTime()
    received_at: float  # Host monotonic receipt

    def __post_init__(self):
        if any(type(value) is not bool for value in (self.started, self.valid, self.completed)):
            raise ValueError("native race flags must be booleans")
        if not all(math.isfinite(value) for value in (self.time_seconds, self.finish_time_seconds, self.received_at)):
            raise ValueError("native race clocks must be finite")
        if type(self.active_gate_index) is not int:
            raise ValueError("native gate index must be an integer")


class RaceStateReader:
    def __init__(self, path, timeout=1.0):
        self.path, self.timeout, self.stream = Path(path), timeout, None

    def poll(self):
        if self.stream is None:
            try:
                if time.time() - self.path.stat().st_mtime > self.timeout:
                    return ()  # An old file cannot authorize an attached run.
                self.stream = self.path.open()
            except FileNotFoundError:
                return ()
        states = []
        while True:
            position = self.stream.tell()
            line = self.stream.readline()
            if not line.endswith("\n"):
                self.stream.seek(position)  # Wait for a complete observation.
                return tuple(states)
            states.append(NativeRaceState(**json.loads(line), received_at=time.monotonic()))

    def close(self):
        if self.stream is not None:
            self.stream.close()
