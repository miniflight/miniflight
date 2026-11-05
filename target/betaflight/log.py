"""Read timestamped gyro rows exported by blackbox_decode."""
import csv
import math
from pathlib import Path


def gyro_rows(path):
    """Yield (loop iteration, device microseconds, gyro degrees/s), preserving gaps."""
    previous = (-1, -1)
    fields = [f"gyroADC[{axis}] (deg/s)" for axis in range(3)]
    with Path(path).open() as source:
        rows = csv.DictReader(source, skipinitialspace=True)
        required = {"loopIteration", "time (us)", *fields}
        if not required.issubset(rows.fieldnames or ()):
            raise ValueError("Expected blackbox_decode CSV with time in us and rotation in deg/s")
        for row in rows:
            iteration, time_us = int(row["loopIteration"]), int(row["time (us)"])
            gyro = tuple(float(row[field]) for field in fields)
            if iteration <= previous[0] or time_us <= previous[1] or not all(map(math.isfinite, gyro)):
                raise ValueError(f"Invalid gyro row at iteration {iteration}")
            yield iteration, time_us, gyro
            previous = iteration, time_us
