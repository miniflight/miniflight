from dataclasses import dataclass
from typing import Generic, TypeVar

from miniflight import Command, Ned, State


CommandT = TypeVar("CommandT", bound=Command, covariant=True)


@dataclass(frozen=True)
class Gate:
    """Gate geometry in metres, with a normalized wxyz orientation in NED.

    center is the derived opening center; origin is the published gate base.
    width and height are reported overall bounds, not opening clearance.
    Older recordings omit origin.
    """

    id: int
    center: Ned
    orientation: tuple[float, float, float, float]
    width: float  # reported overall width in metres
    height: float  # reported overall height in metres
    origin: Ned | None = None


class BaseController(Generic[CommandT]):
    def update(self, state: State, gate_index: int, gates: tuple[Gate, ...] | None) -> CommandT | None:
        """Return one vehicle command, or None while waiting for required observations.

        CommandT is the output plane, or a union for a routine that switches planes.
        The target validates each returned command before it is sent.

        state: A fresh IMU sample plus the latest optional observations, each with its
               own timestamp. Unavailable or stale optional samples are None.
        gate_index: The zero-based active gate; len(gates) means all gates were passed.
                    The simulator checks its bounds when track geometry is available.
        gates: Cached geometry for the whole track, not a per-cycle sensor sample.
               None until a usable complete transfer arrives. Partial replacements
               leave the previous track available.
        """
        raise NotImplementedError
