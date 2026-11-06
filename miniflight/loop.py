"""Run control from asynchronous signals; the external system owns physics."""

import math
import time
from dataclasses import dataclass


@dataclass
class Cadence:
    period: float
    deadline: float

    def next_tick(self, now):
        scheduled = self.deadline + self.period
        self.deadline = now + self.period if scheduled < now else scheduled
        return self.deadline


def run(receive, update, send, *, hz=50, clock=time):
    """Receive timestamped inputs, compute a command, send through the adapter.

    None from receive waits for input. StopIteration ends control; the caller
    owns connection lifetime and shutdown. Receiving does not step physics.
    clock supplies monotonic() and sleep(); live execution uses Python time.
    """
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("control frequency must be finite and positive")
    period = 1 / hz
    now = clock.monotonic()
    cadence = Cadence(period, now - period)
    while True:
        deadline = cadence.next_tick(now)
        clock.sleep(max(0, deadline - clock.monotonic()))
        signals = receive(clock.monotonic())
        if signals is not None:
            send(update(*signals))
        now = clock.monotonic()
