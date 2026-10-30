"""Attitude estimates from body-frame measurements."""

import math

from common.math import Quaternion


def stationary_down(acceleration):
    """Unit down vector in body axes, assuming zero translational acceleration.

    Input is specific force in m/s². This cannot determine yaw or detect rest.
    """
    magnitude = math.hypot(*acceleration)
    if not math.isfinite(magnitude) or magnitude == 0:
        return None
    return tuple(-value / magnitude for value in acceleration)


def gyro_step(rotation, gyro, dt):
    """Advance a body-to-local quaternion using constant FRD rad/s for dt seconds."""
    speed = math.hypot(*gyro)
    if dt < 0 or not all(math.isfinite(v) for v in (speed, dt)):
        raise ValueError("gyro and elapsed seconds must be finite; dt must be nonnegative")
    if speed == 0 or dt == 0:
        return rotation
    advanced = rotation * Quaternion.from_axis_angle(gyro, speed * dt)
    advanced.normalize()
    return advanced
