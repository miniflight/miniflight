"""Attitude estimates from body-frame measurements."""

import math

from common.math import Quaternion
from miniflight.state import RateSample


def stationary_down(acceleration):
    """Unit down vector in body axes, assuming zero translational acceleration.

    Input is specific force in m/s². This cannot determine yaw or detect rest.
    """
    magnitude = math.hypot(*acceleration)
    if not math.isfinite(magnitude) or magnitude == 0:
        return None
    return tuple(-value / magnitude for value in acceleration)


def stationary_rotation(acceleration):
    """Body-to-local rotation from stationary specific force; yaw is set to zero."""
    down = stationary_down(acceleration)
    if down is None:
        return None
    x, y, z = down
    return Quaternion.from_euler(math.atan2(y, z), math.atan2(-x, math.hypot(y, z)), 0)


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


class GyroAttitude:
    """Carry a supplied body-to-local rotation through timestamped FRD gyro samples."""

    def __init__(self, rotation, sample: RateSample):
        if rotation is None or sample is None or sample.time is None or sample.gyro is None:
            raise ValueError("an initial rotation and timestamped gyro are required")
        self.rotation = rotation
        self.time, self.gyro = sample.time, sample.gyro

    def update(self, sample: RateSample | None):
        if sample is None or sample.time is None or sample.gyro is None or sample.time == self.time:
            return self.rotation
        gyro = tuple((a + b) / 2 for a, b in zip(self.gyro, sample.gyro, strict=True))
        self.rotation = gyro_step(self.rotation, gyro, sample.time - self.time)
        self.time, self.gyro = sample.time, sample.gyro
        return self.rotation
