from miniflight import Command, State, Vehicle
from target.aigp import SimulatorClient


class BaseController:
    """One vehicle connection and the AI-GP race signals for a control loop."""

    def __init__(self, port=14550, camera_port=5600):
        self.client = SimulatorClient(port=port, camera_port=camera_port)
        self.vehicle = Vehicle(self.client)

    @property
    def race(self):
        return self.client.race

    def update(self, state: State) -> Command | None:
        raise NotImplementedError
