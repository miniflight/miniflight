from dataclasses import dataclass

from miniflight import Command, Ned, State


@dataclass(frozen=True)
class Gate:
    """Track geometry: center in NED metres, orientation as a wxyz quaternion."""

    id: int
    center: Ned
    orientation: tuple[float, float, float, float]
    width: float
    height: float


class BaseController:
    def update(self, state: State, gate_index: int, gates: tuple[Gate, ...] | None) -> Command | None:
        """Return one vehicle command, or None while waiting for required observations.

        state: Current vehicle observations. Unavailable or stale optional samples are None.
        gate_index: The zero-based active gate reported by the simulator.
        gates: The complete track geometry, or None when it is unavailable.
        """
        raise NotImplementedError
