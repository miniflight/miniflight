"""Offline race planning with free crossing positions and interval durations."""

import numpy as np


def integrate_motion(initial, jerk, intervals):
    """Integrate piecewise constant NED jerk; states are position/velocity/acceleration."""
    position, velocity, acceleration = initial
    dt = np.asarray(intervals)[:, None]
    accelerations = np.vstack((acceleration, acceleration + np.cumsum(jerk * dt, axis=0)))
    velocity_changes = accelerations[:-1] * dt + jerk * dt**2 / 2
    velocities = np.vstack((velocity, velocity + np.cumsum(velocity_changes, axis=0)))
    position_changes = velocities[:-1] * dt + accelerations[:-1] * dt**2 / 2 + jerk * dt**3 / 6
    positions = np.vstack((position, position + np.cumsum(position_changes, axis=0)))
    return np.stack((positions, velocities, accelerations), axis=1)
