"""Offline race planning with free crossing positions and interval durations."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize


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
    dense = integrate_motion(initial, np.repeat(jerk, 4, axis=0), np.repeat(intervals / 4, 4))
    _, velocity, acceleration = dense.transpose(1, 0, 2)
    force = force_for_motion(velocity, acceleration, model["quadratic_drag_per_m"])
    horizontal_force_squared = np.sum(force[:, :2]**2, axis=1)
    force_squared = np.sum(force**2, axis=1)
    minimum_thrust, maximum_thrust = model["specific_thrust_range_m_s2"]
    force_limit = model["horizontal_force_limit_m_s2"]
    up = -force[:, 2]
    direction = -force / np.maximum(np.linalg.norm(force, axis=1)[:, None], 1e-12)
    direction_rate_squared = np.sum(np.diff(direction, axis=0)**2, axis=1) / np.repeat(intervals / 4, 4)**2
    down_demand = (force[:, 2] + 9.81) / model["vertical_force_limit_m_s2"]
    down_speed = velocity[:, 2] / model["vertical_speed_limit_m_s"]
    down_acceleration = acceleration[:, 2] / model["vertical_acceleration_limit_m_s2"]
    depth_margin = [] if "maximum_down_m" not in model else model["maximum_down_m"] - dense[:, 0, 2]
    return np.r_[force_squared/minimum_thrust**2-1, 1-force_squared/maximum_thrust**2, 1-horizontal_force_squared/force_limit**2,
                 (up**2*np.tan(model["tilt_limit_rad"])**2-horizontal_force_squared)/force_limit**2, up/9.81,
                 1-down_demand, 1+down_demand, 1-down_speed, 1+down_speed, 1-down_acceleration, 1+down_acceleration,
                 1-np.sum(velocity[:, :2]**2, axis=1)/model["horizontal_speed_limit_m_s"]**2,
                 1-direction_rate_squared/model["direction_rate_limit_rad_s"]**2, depth_margin]


def crossing_constraints(states, gates, centers, model):
    """Plane equality and clearance/direction margins in the measured R1 gate frame."""
    axes = np.asarray(model["gate_axes_ned"])
    offsets = (states[gates, 0] - centers) @ axes.T
    phase_lengths = np.linalg.norm(np.diff(np.vstack((states[0, 0], centers)), axis=0), axis=1)
    plane_error = offsets[:, 1] / phase_lengths
    in_plane = offsets[:, (0, 2)] / model["clearance_halfspan_m"]
    crossing_speed = states[gates, 1] @ axes[1]
    return plane_error, np.r_[(1-in_plane).ravel(), (1+in_plane).ravel(), crossing_speed]


def plan(reference, model, *, callback=None, max_iterations=500):
    """Minimize duration with SLSQP; gate positions and six phase durations remain free."""
    initial, seed_jerk = reference["states"][0], reference["acceleration_rates"]
    gates = reference["gates"]
    centers = reference["gate_centers"] if "gate_centers" in reference else reference["points"][1:]
    counts = np.diff(np.r_[0, gates])
    phases = len(gates)
    crossing_times = reference["times"][gates] if "times" in reference else gates * float(reference["dt"])
    seed_durations = np.diff(np.r_[0, crossing_times])
    seed = np.r_[seed_durations, seed_jerk.ravel()]
    def motion(variables):
        durations, jerk = variables[:phases], variables[phases:].reshape(-1, 3)
        intervals = np.repeat(durations / counts, counts)
        states = integrate_motion(initial, jerk, intervals)
        return states, jerk, intervals
    def equality(variables):
        states, _, _ = motion(variables)
        planes, _ = crossing_constraints(states, gates, centers, model)
        return np.r_[planes, states[-1, 2]]
    def inequality(variables):
        states, jerk, intervals = motion(variables)
        _, clearance = crossing_constraints(states, gates, centers, model)
        dynamics = motion_constraints(initial, jerk, intervals, model)
        departure = model["departure_down_limit_m"] - states[:gates[0]+1, 0, 2]
        return np.r_[clearance, dynamics, departure]
    gradient = np.r_[np.ones(phases), np.zeros(seed_jerk.size)]
    jerk_limit = model["jerk_limit_m_s3"]
    result = minimize(lambda variables: variables[:phases].sum(), seed, method="SLSQP", jac=lambda _: gradient,
                      bounds=[(.5, 20)]*phases + [(-jerk_limit, jerk_limit)]*seed_jerk.size,
                      constraints=[dict(type="eq", fun=equality), dict(type="ineq", fun=inequality)], callback=callback,
                      options=dict(ftol=1e-6, eps=1e-6, maxiter=max_iterations))
    equality_error, minimum_margin = abs(equality(result.x)).max(), inequality(result.x).min()
    if not result.success or equality_error > 1e-5 or minimum_margin < -1e-5:
        raise ValueError(f"{result.message}; equality {equality_error:g}, margin {minimum_margin:g}")
    states, jerk, intervals = motion(result.x)
    times = np.r_[0, np.cumsum(intervals)]
    return dict(states=states, acceleration_rates=jerk, times=times, dt=intervals.mean(), gates=gates,
                points=np.vstack((states[0, 0], states[gates, 0])), gate_centers=centers, iterations=result.nit,
                equality_error=equality_error, minimum_margin=minimum_margin)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--max-iterations", type=int, default=500)
    args = parser.parse_args()
    model = json.loads(args.model.read_text())
    with np.load(args.reference) as reference:
        result = plan(reference, model, max_iterations=args.max_iterations)
    with args.output.open("xb") as output:
        np.savez(output, **result)
    print(json.dumps(dict(duration_s=float(result["times"][-1]), iterations=int(result["iterations"]),
                          crossing_times_s=result["times"][result["gates"]].tolist())))


if __name__ == "__main__":
    main()
