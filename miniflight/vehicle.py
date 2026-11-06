from enum import Enum, auto

from miniflight.control import BodyRates, Command, PositionNed, VelocityNed
from miniflight.state import Attitude, Frame, Motion, MotorOutputs, Ned, State  # preserve existing imports


class Binding(Enum):
    UNBOUND = auto()
    BOUND = auto()


class Vehicle:
    """Read observations and send commands through one target connection."""

    def __init__(self, target=None) -> None:
        self._target = target

    def bind(self, target) -> None:
        """Select the backend once, before running the vehicle program."""
        if self.binding is Binding.BOUND:
            raise RuntimeError("vehicle already has a target")
        self._target = target

    @property
    def binding(self):
        return Binding.UNBOUND if self._target is None else Binding.BOUND

    def _bound_target(self):
        if self.binding is Binding.UNBOUND:
            raise RuntimeError("vehicle has no target")
        return self._target

    def connect(self) -> None:
        self._bound_target().connect()

    def disconnect(self) -> None:
        try:
            self._bound_target().disconnect()
        finally:
            self._target = None

    @property
    def state(self) -> State | None:
        return None if self._target is None else self._target.state

    @property
    def commands(self) -> frozenset[type]:
        return self._bound_target().commands

    def read(self, timeout=1.0) -> State | None:
        state = self._bound_target().read(timeout=timeout)
        if state is not None and not isinstance(state, State):
            raise TypeError("target.read() must return State or None")
        return state

    def validate(self, command: Command) -> None:
        """Check support without sending or arming."""
        if not isinstance(command, Command):
            raise TypeError("expected a flight command")
        if type(command) not in self.commands:
            raise NotImplementedError(f"target does not support {type(command).__name__}")

    def send(self, command: Command) -> None:
        self.validate(command)
        self._bound_target().send(command)

    @property
    def position(self) -> Ned | None:
        """Cached position; call read() to receive another observation."""
        return self.state.motion.position if self.state and self.state.motion else None

    @property
    def velocity(self) -> Ned | None:
        return self.state.motion.velocity if self.state and self.state.motion else None

    def arm(self) -> None:
        self._bound_target().arm()

    def disarm(self) -> None:
        self._bound_target().disarm()

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


vehicle = Vehicle()
