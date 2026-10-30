"""Attitude estimates from body-frame measurements."""

import math


def stationary_down(acceleration):
    """Unit down vector in body axes, assuming zero translational acceleration.

    Input is specific force in m/s². This cannot determine yaw or detect rest.
    """
    magnitude = math.hypot(*acceleration)
    if not math.isfinite(magnitude) or magnitude == 0:
        return None
    return tuple(-value / magnitude for value in acceleration)
