from dataclasses import dataclass
import math

from target import Target


@dataclass(frozen=True)
class PositionNED:
    north: float
    east: float
    down: float


@dataclass(frozen=True)
class VelocityNED:
    north: float
    east: float
    down: float


class Vehicle:
    def __init__(self, target: Target) -> None:
        self._target = target

    def connect(self) -> None:
        self._target.connect()

    def disconnect(self) -> None:
        self._target.disconnect()

    @property
    def position(self) -> PositionNED:
        return PositionNED(*self._target.position())

    @property
    def velocity(self) -> VelocityNED:
        return VelocityNED(*self._target.velocity())

    def arm(self) -> None:
        self._target.arm()

    def disarm(self) -> None:
        self._target.disarm()

    def position_ned(
        self,
        north: float,
        east: float,
        down: float,
    ) -> None:
        if not all(math.isfinite(value) for value in (north, east, down)):
            raise ValueError("position must be finite")

        self._target.position_ned(north, east, down)

    def velocity_ned(
        self,
        north: float,
        east: float,
        down: float,
    ) -> None:
        if not all(math.isfinite(value) for value in (north, east, down)):
            raise ValueError("velocity must be finite")

        self._target.velocity_ned(north, east, down)

    def body_rates(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        thrust: float,
    ) -> None:
        values = (roll, pitch, yaw, thrust)

        if not all(math.isfinite(value) for value in values):
            raise ValueError("body rates and thrust must be finite")

        if not 0.0 <= thrust <= 1.0:
            raise ValueError("thrust must be between 0 and 1")

        self._target.body_rates(roll, pitch, yaw, thrust)
