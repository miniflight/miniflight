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


def attitude_rows(path):
    """Yield relative time, loop index, FRD gyro deg/s and body-to-start quaternion."""
    from common.math import Quaternion
    from miniflight.attitude import GyroAttitude
    from miniflight.state import RateSample

    observer = None
    for iteration, time_us, (x, y, z) in gyro_rows(path):
        if observer is None:
            origin, previous = time_us, iteration-1
        if iteration != previous+1:
            raise ValueError(f"Missing loop iterations before {iteration}")
        # Betaflight's FLU body axes become Miniflight's forward/right/down axes.
        gyro = (x, -y, -z)
        sample = RateSample((time_us-origin)*1e-6, 0., tuple(math.radians(v) for v in gyro))
        if observer is None:
            observer = GyroAttitude(Quaternion(), sample)
        rotation = observer.update(sample)
        yield sample.time, iteration, gyro, tuple(rotation.q)
        previous = iteration
