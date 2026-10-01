from miniflight.control import BodyRates, Command, PositionNed, VelocityNed
from miniflight.state import Attitude, Frame, Motion, MotorOutputs, Ned, State  # preserve existing imports


class Vehicle:
    """Read observations and send commands through one target connection."""

    def __init__(self, target) -> None:
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
