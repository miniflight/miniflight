"""Sample planned position, velocity and acceleration in one shared frame."""

from bisect import bisect_right
import math


def sample_motion(states, acceleration_rates, dt, time, *, times=None):
    """States are (position, velocity, acceleration); times are increasing knots from zero."""
    if not math.isfinite(time) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("reference time must be finite and its interval positive")
    time = max(0.0, time)
    if times is None:
        index = min(int(time / dt), len(acceleration_rates))
        elapsed = time - index * dt
    else:
        index = min(bisect_right(times, time) - 1, len(acceleration_rates))
        elapsed = time - times[index]
    position, velocity, acceleration = states[index]
    rate = acceleration_rates[index] if index < len(acceleration_rates) else acceleration * 0
    return (position + velocity * elapsed + acceleration * elapsed**2 / 2 + rate * elapsed**3 / 6,
            velocity + acceleration * elapsed + rate * elapsed**2 / 2,
            acceleration + rate * elapsed)
