"""Sample planned position, velocity and acceleration in one shared frame."""

import math


def sample_motion(states, acceleration_rates, dt, time):
    """States are (position, velocity, acceleration); time and dt are seconds."""
    if not math.isfinite(time) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("reference time must be finite and its interval positive")
    time = max(0.0, time)
    index = min(int(time / dt), len(acceleration_rates))
    elapsed = time - index * dt
    position, velocity, acceleration = states[index]
    rate = acceleration_rates[index] if index < len(acceleration_rates) else acceleration * 0
    return (position + velocity * elapsed + acceleration * elapsed**2 / 2 + rate * elapsed**3 / 6,
            velocity + acceleration * elapsed + rate * elapsed**2 / 2,
            acceleration + rate * elapsed)
