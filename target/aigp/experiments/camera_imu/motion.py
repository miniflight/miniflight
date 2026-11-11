"""Gate-relative camera velocity, with its effective measurement time."""

import numpy as np


class GateMotion:
    def __init__(self, window=.25):
        self.window = window
        self.observations = []

    def observe(self, stamp, position):
        if self.observations and stamp <= self.observations[-1][0]:
            return None
        self.observations.append((stamp, np.array(position, dtype=float)))
        self.observations = [row for row in self.observations if row[0] >= stamp-self.window]
        if len(self.observations) < 4 or stamp-self.observations[0][0] < .15:
            return None
        times = np.array([row[0]-stamp for row in self.observations])
        positions = np.array([row[1] for row in self.observations])
        design = np.column_stack((np.ones(len(times)), times))
        weights = np.ones(len(times))
        for _ in range(4):
            root = np.sqrt(weights)
            fit = np.linalg.lstsq(design*root[:, None], positions*root[:, None], rcond=None)[0]
            residual = np.linalg.norm(design @ fit-positions, axis=1)
            weights = np.minimum(1., max(.02, 1.5*np.median(residual))/np.maximum(residual, 1e-12))
        root = np.sqrt(weights)
        fit = np.linalg.lstsq(design*root[:, None], positions*root[:, None], rcond=None)[0]
        centered = times-np.average(times, weights=weights)
        # A linear fit to constant-acceleration motion measures velocity at this time.
        effective = np.average(times, weights=weights)+np.sum(weights*centered**3)/(2*np.sum(weights*centered**2))
        return dict(time=float(stamp+effective), velocity=fit[1].tolist(),
                    window_s=float(-times[0]), observations=len(times),
                    residual_m=float(np.sqrt(np.average(np.sum((design @ fit-positions)**2, axis=1), weights=weights))))
