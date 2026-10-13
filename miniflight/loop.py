"""Run control from asynchronous signals; the external system owns physics."""

import math
import time


def ticks(hz):
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("control frequency must be finite and positive")
    period = 1 / hz
    now = time.monotonic()
    deadline = now - period
    while True:
        deadline += period
        if deadline < now:
            deadline = now + period
        time.sleep(max(0, deadline - time.monotonic()))
        yield time.monotonic()
        now = time.monotonic()
