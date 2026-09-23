from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from target import Target
from miniflight.control import BodyRates, Command, PositionNed, VelocityNed


class Ned(NamedTuple):
    """North, east, down components; units belong to the containing field."""

    north: float
    east: float
    down: float


@dataclass(frozen=True)
class Motion:
    time: float  # device seconds
    received_at: float  # host monotonic seconds
    position: Ned  # metres in the target's local NED frame
    velocity: Ned  # m/s in the same frame


@dataclass(frozen=True)
class Attitude:
    time: float
    received_at: float
    roll: float  # radians, FRD body relative to local NED
    pitch: float
    yaw: float


@dataclass(frozen=True)
class MotorOutputs:
    """Reported output channels in target-defined units, not measured RPM."""

    time: float
    received_at: float
    outputs: tuple[float, ...]
    active: int  # channel bitmask


@dataclass(frozen=True)
class Frame:
    id: int
    time_ns: int  # device timestamp
    received_at: float
    bgr: np.ndarray


@dataclass(frozen=True)
class State:
    """Latest observations at an IMU update, not a synchronized ground truth."""

    time: float  # device IMU timestamp in seconds
    dt: float  # device seconds since the previous read; first read is zero
    acceleration: tuple[float, float, float]  # reported FRD body acceleration, m/s²
    gyro: tuple[float, float, float]  # FRD body angular velocity, rad/s
    received_at: float  # IMU receipt time, host monotonic seconds
    frame: Frame | None = None
    motion: Motion | None = None
    attitude: Attitude | None = None
    motors: MotorOutputs | None = None


class Vehicle:
    """Read observations and send commands through one target connection."""

    def __init__(self, target: Target) -> None:
        self._target = target
        self._state = None

    def connect(self) -> None:
        self._target.connect()
        self._state = None

    def disconnect(self) -> None:
        try:
            self._target.disconnect()
        finally:
            self._state = None

    @property
    def state(self) -> State | None:
        return self._state

    @property
    def commands(self) -> frozenset[type]:
        return self._target.commands

    def read(self, timeout=1.0) -> State:
        state = self._target.read(timeout=timeout)
        if not isinstance(state, State):
            raise TypeError("target.read() must return State")
        self._state = state
        return state

    def validate(self, command: Command) -> None:
        """Check support without sending or arming."""
        if not isinstance(command, Command):
            raise TypeError("expected BodyRates, PositionNed or VelocityNed")
        if type(command) not in self.commands:
            raise NotImplementedError(f"target does not support {type(command).__name__}")

    def send(self, command: Command) -> None:
        self.validate(command)
        self._target.send(command)

    @property
    def position(self) -> Ned | None:
        """Cached position; call read() to receive another observation."""
        return self._state.motion.position if self._state and self._state.motion else None

    @property
    def velocity(self) -> Ned | None:
        return self._state.motion.velocity if self._state and self._state.motion else None

    def arm(self) -> None:
        self._target.arm()

    def disarm(self) -> None:
        self._target.disarm()

    def position_ned(self, north: float, east: float, down: float) -> None:
        self.send(PositionNed(north, east, down))

    def velocity_ned(self, north: float, east: float, down: float) -> None:
        self.send(VelocityNed(north, east, down))

    def body_rates(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        thrust: float,
    ) -> None:
        self.send(BodyRates(roll, pitch, yaw, thrust))
