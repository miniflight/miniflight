"""Body-rate feedback, independent of the airframe and transport."""

import math


def rate_control(kp, ki, integral, target, measured, dt):
    """FRD rate PI: rad/s error -> (torque effort, next integral effort).

    kp has units of effort/(rad/s), ki of effort/rad, and dt is seconds.
    The caller owns integral state and handles actuator limits after mixing.
    """
    vectors = (kp, ki, integral, target, measured)
    if any(len(v) != 3 for v in vectors) or not all(math.isfinite(x) for v in vectors for x in v):
        raise ValueError("rate feedback requires finite three-component vectors")
    if any(k <= 0 for k in kp) or any(k < 0 for k in ki) or not math.isfinite(dt) or dt < 0:
        raise ValueError("rate gains and elapsed time are invalid")
    error = tuple(wanted - actual for wanted, actual in zip(target, measured))
    integral = tuple(previous + k * e * dt for previous, k, e in zip(integral, ki, error))
    return tuple(k * e + i for k, e, i in zip(kp, error, integral)), integral
