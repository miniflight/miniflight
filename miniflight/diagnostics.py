"""Read complete diagnostic records independently of a controller or target."""

import json
from pathlib import Path


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
