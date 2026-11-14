"""Camera/IMU estimation primitives without simulator state."""

from bisect import bisect_right

import numpy as np


class ImuIntegral:
    """Integrate supplied world-frame specific force using measured physics dt.

    Publication timestamps locate samples; cumulative measured dt carries the
    integration duration. Gravity is added by the caller after differencing.
    """
    def __init__(self):
        self.times, self.rows = [], []

    def append(self, stamp, dt, force):
        if dt <= 0 or (self.times and stamp <= self.times[-1]):
            raise ValueError('IMU increments must advance with positive dt')
        if not self.rows:
            self.times.append(stamp-dt)
            self.rows.append(np.zeros(7))
        elapsed, vx, vy, vz, px, py, pz = self.rows[-1]
        velocity = np.array([vx, vy, vz])
        position = np.array([px, py, pz])
        force = np.asarray(force)
        self.times.append(stamp)
        self.rows.append(np.r_[elapsed+dt, velocity+force*dt, position+velocity*dt+.5*force*dt**2])

    def at(self, stamp):
        if not self.times or not self.times[0] <= stamp <= self.times[-1]:
            raise ValueError('camera time is outside the IMU history')
        index = bisect_right(self.times, stamp)-1
        if index+1 == len(self.times):
            return self.rows[index].copy()
        fraction = (stamp-self.times[index])/(self.times[index+1]-self.times[index])
        return self.rows[index]+fraction*(self.rows[index+1]-self.rows[index])

    def between(self, first, last):
        a, b = self.at(first), self.at(last)
        dt = b[0]-a[0]
        return dt, b[1:4]-a[1:4], b[4:7]-a[4:7]-a[1:4]*dt
