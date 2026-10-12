"""Read complete diagnostic records independently of a controller or target."""

import json
from pathlib import Path


def write_snapshot(path, data):
    """Publish one complete view; readers see the previous or next JSON object."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, allow_nan=False) + "\n")
    temporary.replace(path)


def chart_data(samples, panels):
    """Map scalar samples into labelled series; no target or control knowledge."""
    time = [sample["time_s"] for sample in samples]
    charts = []
    for title, unit, channels in panels:
        series = []
        for label, key in channels:
            series.append(dict(label=label, x=time, y=[sample.get(key) for sample in samples]))
        charts.append(dict(title=title, unit=unit, series=series))
    return charts


class JsonLines:
    def __init__(self, path):
        self.path, self.stream = Path(path), None

    def poll(self):
        if self.stream is None:
            try:
                self.stream = self.path.open()
            except FileNotFoundError:
                return []
        records = []
        while True:
            position = self.stream.tell()
            line = self.stream.readline()
            if not line.endswith("\n"):
                self.stream.seek(position)
                return records
            records.append(json.loads(line))

    def close(self):
        if self.stream is not None:
            self.stream.close()
