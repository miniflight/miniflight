from miniflight import Control, State
from target.aigp.controllers import BaseController


class Controller(BaseController):
    def update(self, state: State) -> Control:
        return Control()
