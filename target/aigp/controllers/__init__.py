from dataclasses import dataclass
import importlib
from typing import Generic, TypeVar, TYPE_CHECKING

from miniflight import Command, Frame, Ned

if TYPE_CHECKING:
    from target.aigp.client import Packet
    from target.aigp.native import NativeRaceState


CommandT = TypeVar("CommandT", bound=Command, covariant=True)


@dataclass(frozen=True)
class Gate:
    """Published NED gate position, wxyz orientation and overall bounds in metres."""

    id: int
    position: Ned  # native actor/mesh pivot
    orientation: tuple[float, float, float, float]
    width: float  # reported overall width in metres
    height: float  # reported overall height in metres


class BaseController(Generic[CommandT]):
    def update(self, telemetry: tuple["Packet", ...], frames: tuple[Frame, ...],
               race_state: "NativeRaceState | None") -> CommandT | None:
        """Read vehicle.state and return a command; AIGP events remain available.

        telemetry retains ordered MAVLink packets, full fields and source clocks.
        Packet.decoded contains RaceTelemetry, or TrackInfo when its transfer completes.
        Packet.privileged labels native pose and course traffic.
        frames contains newly completed BGR images, each with its own timestamp.
        Empty tuples mean no new arrivals. Vehicle exposes the accepted samples;
        the controller owns control history, such as integrals and references.
        Inputs arrive before GO and at finish; the runner gates command sending.
        race_state contains native GO, validity and separately derived mesh centers;
        None means it is unavailable. Published Gate fields remain unchanged.
        """
        raise NotImplementedError


def load_controller(name):
    module = name if "." in name else f"target.aigp.controllers.{name}"
    controller = getattr(importlib.import_module(module), "Controller", None)
    if not isinstance(controller, type) or not issubclass(controller, BaseController):
        raise TypeError(f"{module} must export Controller(BaseController)")
    return controller


def update_controller(controller, telemetry, frames, race_state):
    if any(packet.data.get_type() == "COLLISION" for packet in telemetry):
        raise RuntimeError("native collision")
    return controller.update(telemetry, frames, race_state)
