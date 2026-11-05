"""Expose recorded AIGP producer samples at the Vehicle.read boundary."""
from dataclasses import replace

from miniflight.state import ImuIncrement
from target.aigp.experiments.recording import ProcessClient


class ProducerImuClient(ProcessClient):
    def __init__(self, calls, producer_read=None, **kwargs):
        super().__init__(calls, **kwargs)
        self.producer_read = producer_read

    def read(self, timeout=1.0):
        state = super().read(timeout)
        batch = self.calls("producer_imu", {}, self.producer_read)
        # Producer rows carry raw AIGP gyro signs, as on the UDP stream.
        increments = tuple(ImuIncrement(row[0]*1e-6, row[2], tuple(row[4:7]), tuple(-g for g in row[7:10]))
                           for row in batch["rows"])
        self.state = replace(state, imu_increments=increments)
        return self.state
