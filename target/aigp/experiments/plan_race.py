"""Offline race planning with free crossing positions and interval durations."""

import numpy as np


def force_for_motion(velocity, acceleration, drag):
    """Required NED thrust per unit mass under the fitted local resistance model."""
    horizontal_speed = np.linalg.norm(velocity[..., :2], axis=-1, keepdims=True)
    horizontal_resistance = drag[0] * horizontal_speed * velocity[..., :2]
    vertical_resistance = drag[2] * abs(velocity[..., 2:]) * velocity[..., 2:]
    resistance = np.concatenate((horizontal_resistance, vertical_resistance), axis=-1)
    required_acceleration = acceleration + resistance
    return required_acceleration - (0, 0, 9.81)


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
