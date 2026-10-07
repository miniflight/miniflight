from dataclasses import dataclass
from typing import Generic, TypeVar

from miniflight import Command, Ned, State


CommandT = TypeVar("CommandT", bound=Command, covariant=True)


@dataclass(frozen=True)
class Gate:
    """Published NED gate position, wxyz orientation and overall bounds in metres."""

    id: int
    position: Ned  # native position is at the gate base
    orientation: tuple[float, float, float, float]
    width: float  # reported overall width in metres
    height: float  # reported overall height in metres


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
