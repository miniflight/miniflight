from miniflight import Control, PositionNed, State
from target.aigp import SimulatorClient


class BaseController(SimulatorClient):
    """AI-GP MAVLink and camera ports with a per-sample control hook."""

    def update(self, state: State) -> Control | PositionNed | None:
        raise NotImplementedError
