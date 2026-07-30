"""The small hardware-neutral contract for Miniflight flight programs.

Programs receive calibrated observations in one declared coordinate system and
return body-motion intent.  The runtime, not a program, owns calibration,
mixing, actuator limits, arming, and motor output.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import math
from dataclasses import dataclass
from typing import FrozenSet, Optional, Protocol

from miniflight.core.observation import ObservationTick


FlightObservation = ObservationTick


@dataclass(frozen=True)
class BodyCommand:
    """Hardware-neutral body-motion intent from a flight program.

    `collective_accel_mps2` is desired upward acceleration along the body up
    axis.  The runtime translates this request through its learned vehicle
    model and applies its own limits.  A program cannot address motors.
    """

    collective_accel_mps2: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        values = (
            self.collective_accel_mps2,
            self.roll_rate_rad_s,
            self.pitch_rate_rad_s,
            self.yaw_rate_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("body-command values must be finite")
        if self.collective_accel_mps2 < 0.0:
            raise ValueError("collective_accel_mps2 must be non-negative")


@dataclass(frozen=True)
class ProgramCapabilities:
    """Facts a runtime declares before a program starts."""

    sensors: FrozenSet[str]
    actuators_enabled: bool
    vehicle_id: Optional[str] = None


class FlightProgram(ABC):
    """A plug-and-play flight behavior.

    The first concrete program will be `hover`.  Programs contain behavior,
    never board access or direct motor writes.
    """

    name: str

    def start(self, capabilities: ProgramCapabilities) -> None:
        """Initialize program state after the runtime describes the vehicle."""

    @abstractmethod
    def step(self, observation: FlightObservation) -> BodyCommand:
        """Return one body command for one fresh canonical observation."""

    def stop(self, reason: str) -> None:
        """Release program-local state.  The runtime disarms separately."""


class ObservationSource(Protocol):
    """Read-only source for canonical observations."""

    def read(self) -> ObservationTick:
        """Return one fresh canonical observation."""


class ReadOnlyProgramRuntime:
    """Run a program against live or replay data without actuator access.

    This is the first runtime mode for a real Betaflight-connected drone.  It
    proves units, axes, timestamps, program behavior, and replay before any
    flight program can request deployment.
    """

    def __init__(
        self,
        source: ObservationSource,
        program: FlightProgram,
        capabilities: ProgramCapabilities,
    ) -> None:
        if capabilities.actuators_enabled:
            raise ValueError("read-only runtime cannot enable actuators")
        self._source = source
        self._program = program
        self._capabilities = capabilities
        self._last_timestamp_ns: Optional[int] = None
        self._started = False
        self.last_command: Optional[BodyCommand] = None

    def start(self) -> None:
        if self._started:
            raise RuntimeError("program runtime already started")
        self._program.start(self._capabilities)
        self._started = True

    def step(self) -> BodyCommand:
        if not self._started:
            raise RuntimeError("start the program runtime before step")
        observation = self._source.read()
        if (
            self._last_timestamp_ns is not None
            and observation.timestamp_ns <= self._last_timestamp_ns
        ):
            raise RuntimeError("observation timestamps must increase")
        self._last_timestamp_ns = observation.timestamp_ns
        self.last_command = self._program.step(observation)
        return self.last_command

    def stop(self, reason: str = "operator") -> None:
        if self._started:
            self._program.stop(reason)
            self._started = False


__all__ = [
    "BodyCommand",
    "FlightObservation",
    "FlightProgram",
    "ObservationSource",
    "ProgramCapabilities",
    "ReadOnlyProgramRuntime",
]
