"""Hold level pitch for two seconds after GO."""

import math

from miniflight import BodyRates
from target.aigp.client import RaceStatus
from target.aigp.controllers import BaseController


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self):
        self.attitude = None
        self.race = None

    def update(self, telemetry, frames) -> BodyRates | None:
        for packet in telemetry:
            if packet.data.get_type() == "ATTITUDE":
                self.attitude = packet.data
            elif isinstance(packet.decoded, RaceStatus):
                self.race = packet.decoded
        if self.race is not None and self.race.started and self.race.sim_boot_time_ms - self.race.race_start_boot_time_ms >= 2000:
            raise StopIteration
        if self.attitude is None:
            return None

        pitch = -self.attitude.pitch  # VQ1 wire angle -> FRD/NED, radians.
        if not math.isfinite(pitch):
            raise ValueError("nonfinite pitch")
        pitch_rate = max(-.75, min(.75, 2.0 * (0.0 - pitch)))
        return BodyRates(pitch_rate=pitch_rate, thrust=.28)
