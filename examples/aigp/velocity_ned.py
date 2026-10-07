"""Request -1 m/s on the NED north axis for two seconds, then zero velocity."""

from miniflight import VelocityNed
from target.aigp.controllers import BaseController
from target.aigp.simulator import AIGPSimulator


class Controller(BaseController[VelocityNed]):
    targets = ("vq1.r1",)

    def __init__(self, track=None):
        self.started_at = None
        self.imu = None

    def update(self, telemetry, frames) -> VelocityNed | None:
        for packet in telemetry:
            if packet.data.get_type() == "HIGHRES_IMU":
                self.imu = packet.data
        if self.imu is None:
            return None
        stamp = self.imu.time_usec * 1e-6
        if self.started_at is None:
            self.started_at = stamp
        elapsed = stamp - self.started_at
        if elapsed < 2:
            return VelocityNed(-1, 0, 0)
        if elapsed < 3:
            return VelocityNed(0, 0, 0)
        raise StopIteration


if __name__ == "__main__":
    AIGPSimulator(Controller, "vq1.r1").rollout()
