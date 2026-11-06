"""Read complete diagnostic records independently of a controller or target."""

import json
from enum import Enum, auto
from pathlib import Path


def write_snapshot(path, data):
    """Publish one complete view; readers see the previous or next JSON object."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, allow_nan=False) + "\n")
    temporary.replace(path)


def chart_data(samples, panels, *, x_key="time_s", x_label="Time (s)"):
    """Map scalar samples into labelled series; no target or control knowledge."""
    time = [sample[x_key] for sample in samples]
    charts = []
    for title, unit, channels in panels:
        series = []
        for label, key in channels:
            series.append(dict(label=label, x=time, y=[sample.get(key) for sample in samples]))
        charts.append(dict(title=title, unit=unit, x_label=x_label, series=series))
    return charts


class ReaderPhase(Enum):
    WAITING_FOR_FILE = auto()
    TAILING = auto()
    CLOSED = auto()


class JsonLines:
    def __init__(self, path):
        self.path, self.stream = Path(path), None
        self.phase = ReaderPhase.WAITING_FOR_FILE

    def poll(self):
        return list(self._records())

    def _records(self):
        if self.phase is ReaderPhase.CLOSED:
            raise ValueError("record reader is closed")
        if self.phase is ReaderPhase.WAITING_FOR_FILE:
            try:
                self.stream = self.path.open()
            except FileNotFoundError:
                return
            self.phase = ReaderPhase.TAILING
        while True:
            position = self.stream.tell()
            line = self.stream.readline()
            if not line.endswith("\n"):
                self.stream.seek(position)
                return
            yield json.loads(line)

    def close(self):
        self.phase = ReaderPhase.CLOSED
        if self.stream is not None:
            self.stream.close()
