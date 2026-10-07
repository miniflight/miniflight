from miniflight import BodyRates
from target.aigp.controllers import BaseController


class Controller(BaseController[BodyRates]):
    def update(self, telemetry, frames) -> BodyRates:
        return BodyRates()
