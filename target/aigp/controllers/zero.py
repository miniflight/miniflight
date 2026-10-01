from miniflight import BodyRates, State
from target.aigp.controllers import BaseController


class Controller(BaseController[BodyRates]):
    def update(self, state: State, gate_index: int, gates) -> BodyRates:
        return BodyRates()
