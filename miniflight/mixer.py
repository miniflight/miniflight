"""Collective thrust and FRD torque effort to requested motor controls."""

import math


def mix(collective, torque, coefficients):
    """Apply one [collective, roll, pitch, yaw] coefficient row per motor.

    Torque is normalized control effort, not angular rate or N m. Its scale
    and motor order belong to the supplied airframe matrix. Outputs are not
    clipped: the caller must resolve saturation before sending MotorControls.
    """
    request = (collective, *torque)
    rows = tuple(tuple(row) for row in coefficients)
    if len(request) != 4 or not all(math.isfinite(value) for value in request):
        raise ValueError("provide finite collective thrust and three torque efforts")
    if not rows or any(len(row) != 4 or not all(math.isfinite(value) for value in row) for row in rows):
        raise ValueError("provide four finite mixing coefficients per motor")
    return tuple(sum(gain * value for gain, value in zip(row, request)) for row in rows)
