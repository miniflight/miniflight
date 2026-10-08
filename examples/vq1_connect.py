"""Print one native position packet from an already-running simulator."""
from target.aigp.controllers import BaseController
from target.aigp.client import SimulatorClient
from target.aigp.simulator import AIGPSimulator


class Observe(BaseController):
    def update(self, telemetry, frames, race_state):
        for packet in telemetry:
            if packet.data.get_type() == "LOCAL_POSITION_NED":
                print(packet.data)
                raise StopIteration
        return None


if __name__ == "__main__":
    AIGPSimulator(Observe, client=SimulatorClient(camera_port=None)).rollout(attach=True)
