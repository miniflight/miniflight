from dataclasses import dataclass
from typing import Generic, TypeVar

from miniflight import Command, Ned, State


CommandT = TypeVar("CommandT", bound=Command, covariant=True)


@dataclass(frozen=True)
class Gate:
    """Published opening geometry: NED center in metres and a wxyz orientation."""

    id: int
    center: Ned
    orientation: tuple[float, float, float, float]
    width: float
    height: float


class BaseController(Generic[CommandT]):
    def update(self, state: State, gate_index: int, gates: tuple[Gate, ...] | None) -> CommandT | None:
        """Return one vehicle command, or None while waiting for required observations.

        CommandT is the output plane, or a union for a routine that switches planes.
        The target validates each returned command before it is sent.

        state: Current vehicle observations. Unavailable or stale optional samples are None.
        gate_index: The zero-based active gate; len(gates) means all gates were passed.
                    The simulator checks its bounds when track geometry is available.
        gates: The complete track geometry, or None when it is unavailable.
        """
        raise NotImplementedError
