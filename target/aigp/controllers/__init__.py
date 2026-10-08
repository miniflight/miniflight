from dataclasses import dataclass
from typing import Generic, TypeVar, TYPE_CHECKING

from miniflight import Command, Frame, Ned

if TYPE_CHECKING:
    from target.aigp.client import Packet


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
    def update(self, telemetry: tuple["Packet", ...], frames: tuple[Frame, ...]) -> CommandT | None:
        """Return a command from new arrivals since the previous call.

        telemetry retains ordered MAVLink packets, full fields and source clocks.
        Packet.decoded exposes native race status or a newly completed course.
        frames contains newly completed BGR images, each with its own timestamp.
        Empty tuples mean no new arrivals. The controller owns retained history.
        Inputs arrive before GO and at finish; the runner gates command sending.
        """
        raise NotImplementedError
