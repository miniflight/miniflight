"""Recompute an R1 reference with bounded acceleration/jerk and lift at launch.

Gate centers and crossing fractions stay fixed. This point-mass calculation
minimizes absolute jerk; native flight must establish tracking and clearance.
"""

import argparse
from pathlib import Path

import numpy as np
from scipy.optimize import linprog


def plan(reference, duration, initial_acceleration=(0, 0, -2), acceleration_limit=2.1, jerk_limit=4):
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    count, gates, points = len(reference["acceleration_rates"]), reference["gates"], reference["points"]
    dt = duration / count
    transition = np.array([[1, dt, dt**2 / 2], [0, 1, dt], [0, 0, 1]])
    effect = np.array([dt**3 / 6, dt**2 / 2, dt])
    motion = np.zeros((count + 1, 3, count))
    initial = np.zeros((count + 1, 3, 3))
    initial[0, 0], initial[0, 2] = points[0], initial_acceleration
    for index in range(count):
        motion[index + 1] = transition @ motion[index]
        motion[index + 1, :, index] = effect
        initial[index + 1] = transition @ initial[index]
    acceleration = np.c_[motion[:, 2], np.zeros((count + 1, count))]
    identity = np.eye(count)
    inequalities = np.vstack([acceleration, -acceleration, np.c_[identity, -identity], np.c_[-identity, -identity]])
    equalities = np.c_[np.vstack([motion[gates, 0], motion[-1, 2]]), np.zeros((len(gates) + 1, count))]
    states, rates = [], []
    for axis in range(3):
        base = initial[:, :, axis]
        result = linprog(np.r_[np.zeros(count), np.ones(count)], A_ub=inequalities,
                         b_ub=np.r_[acceleration_limit - base[:, 2], acceleration_limit + base[:, 2], np.zeros(2 * count)],
                         A_eq=equalities, b_eq=np.r_[points[1:, axis] - base[gates, 0], -base[-1, 2]],
                         bounds=[(-jerk_limit, jerk_limit)] * count + [(0, None)] * count, method="highs")
        if not result.success:
            raise ValueError(f"axis {axis}: {result.message}")
        rates.append(result.x[:count])
        states.append(base + motion @ rates[-1])
    return dict(states=np.stack(states, axis=2), acceleration_rates=np.stack(rates, axis=1), dt=dt, gates=gates, points=points)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, required=True)
    args = parser.parse_args()
    with np.load(args.source) as reference:
        result = plan(reference, args.duration)
    with args.output.open("xb") as output:
        np.savez(output, **result)
