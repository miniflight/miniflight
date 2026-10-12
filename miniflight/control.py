from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BodyRates:
    """FRD body rates in rad/s and collective thrust in [0, 1]."""

    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    thrust: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (
            self.roll_rate, self.pitch_rate, self.yaw_rate, self.thrust
        )):
            raise ValueError("body rates and thrust must be finite")
        if not 0.0 <= self.thrust <= 1.0:
            raise ValueError("thrust must be between 0 and 1")


@dataclass(frozen=True)
class PositionNed:
    """Local NED position in metres; the target closes the position loop."""

    north: float
    east: float
    down: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.north, self.east, self.down)):
            raise ValueError("NED setpoints must be finite")


@dataclass(frozen=True)
class VelocityNed:
    """Local NED velocity in m/s; the target closes the velocity loop."""

    north: float
    east: float
    down: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.north, self.east, self.down)):
            raise ValueError("NED setpoints must be finite")


@dataclass(frozen=True)
class MotorControls:
    """Nonreversing motor commands in [0, 1], in the target's motor order; not RPM."""

    outputs: tuple[float, ...]

    def __post_init__(self):
        outputs = tuple(self.outputs)
        if not outputs or any(not math.isfinite(v) or not 0 <= v <= 1 for v in outputs):
            raise ValueError("motor controls must be finite values between 0 and 1")
        object.__setattr__(self, "outputs", outputs)


Command = BodyRates | PositionNed | VelocityNed | MotorControls
