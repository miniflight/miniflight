"""Record common Miniflight observations; observing does no file I/O."""

import csv
from pathlib import Path

from miniflight.state import Motion


class Monitor:
    def __init__(self):
        self.motion: list[Motion] = []

    def observe(self, motion: Motion):
        self.motion.append(motion)

    def save(self, path):
        """Retain independent device and host clocks, NED metres and m/s."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("device_time_s", "host_received_at_s", "north_m", "east_m", "down_m",
                             "north_m_s", "east_m_s", "down_m_s"))
            writer.writerows((sample.time, sample.received_at, *sample.position, *sample.velocity)
                             for sample in self.motion)
        return path
