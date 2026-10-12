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


def motion_constraints(initial, jerk, intervals, model):
    """Nonnegative margins for force, speed, tilt and thrust-direction slew."""
    dense = integrate_motion(initial, np.repeat(jerk, 2, axis=0), np.repeat(intervals / 2, 2))
    _, velocity, acceleration = dense.transpose(1, 0, 2)
    force = force_for_motion(velocity, acceleration, model["quadratic_drag_per_m"])
    horizontal_force_squared = np.sum(force[:, :2]**2, axis=1)
    force_limit = model["horizontal_force_limit_m_s2"]
    up = -force[:, 2]
    direction = -force / np.maximum(np.linalg.norm(force, axis=1)[:, None], 1e-12)
    direction_rate_squared = np.sum(np.diff(direction, axis=0)**2, axis=1) / np.repeat(intervals / 2, 2)**2
    down_demand = (force[:, 2] + 9.81) / model["vertical_force_limit_m_s2"]
    down_speed = velocity[:, 2] / model["vertical_speed_limit_m_s"]
    down_acceleration = acceleration[:, 2] / model["vertical_acceleration_limit_m_s2"]
    return np.r_[1-horizontal_force_squared/force_limit**2,
                 (up**2*np.tan(model["tilt_limit_rad"])**2-horizontal_force_squared)/force_limit**2, up/9.81,
                 1-down_demand, 1+down_demand, 1-down_speed, 1+down_speed, 1-down_acceleration, 1+down_acceleration,
                 1-np.sum(velocity[:, :2]**2, axis=1)/model["horizontal_speed_limit_m_s"]**2,
                 1-direction_rate_squared/model["direction_rate_limit_rad_s"]**2]
