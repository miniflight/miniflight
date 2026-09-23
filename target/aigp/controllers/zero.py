from miniflight import BodyRates, State
from target.aigp.controllers import BaseController


class Controller(BaseController):
    def update(self, state: State) -> BodyRates:
        return BodyRates()
