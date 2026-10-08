from miniflight import BodyRates
from target.aigp.controllers import BaseController


class Controller(BaseController[BodyRates]):
    def __init__(self, track=None):
        super().__init__(track)
        self.motion = None

    def update(self, telemetry, frames) -> BodyRates | None:
        for packet in telemetry:
            if packet.data.get_type() == "LOCAL_POSITION_NED":
                self.motion = packet
        return None
