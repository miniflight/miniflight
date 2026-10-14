"""Run control from asynchronous signals; the external system owns physics."""

import math
import time


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
    deadline = now - period
    while True:
        deadline += period
        if deadline < now:
            deadline = now + period
        clock.sleep(max(0, deadline - clock.monotonic()))
        signals = receive(clock.monotonic())
        if signals is not None:
            send(update(*signals))
        now = clock.monotonic()
