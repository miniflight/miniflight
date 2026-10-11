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

    def plot(self, path, references=(), title="Reported position", reference_label="Reference positions"):
        """Plot NED position and its device clock; references use the same frame."""
        import numpy as np
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        data = np.asarray([(sample.time, *sample.position) for sample in self.motion], dtype=float).reshape(-1, 4)
        references = np.asarray(references, dtype=float).reshape(-1, 3)
        figure = Figure(figsize=(12, 7), layout="constrained")
        FigureCanvasAgg(figure)
        grid = figure.add_gridspec(3, 2, wspace=.15)
        trajectory = figure.add_subplot(grid[:, 0], projection="3d")
        trajectory.plot(*data[:, 1:].T, label="Reported position", linewidth=1.5)
        if len(references):
            trajectory.scatter(*references.T, color="tab:orange", marker="x", s=45, label=reference_label)
            for index, point in enumerate(references, 1):
                trajectory.text(*point, str(index), fontsize=9)
        trajectory.set_box_aspect((2, 1, 1))
        trajectory.text2D(.05, .97, "3D axes use independent scales", transform=trajectory.transAxes, fontsize=8)
        if not np.isfinite(data[:, 1:]).all(axis=1).any():
            trajectory.text2D(.05, .92, "No finite position samples", transform=trajectory.transAxes)
        trajectory.set(xlabel="North (m)", ylabel="East (m)", zlabel="Down (m)")
        trajectory.invert_zaxis()
        trajectory.legend(loc="lower left", fontsize=8)
        for index, label in enumerate(("North (m)", "East (m)", "Down (m)")):
            axis = figure.add_subplot(grid[index, 1])
            axis.plot(data[:, 0], data[:, index + 1], linewidth=1.2)
            axis.set_ylabel(label)
            axis.grid(alpha=.25)
            if index == 2:
                axis.set_xlabel("Device time (s)")
        figure.suptitle(title)
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=150)
        return path
