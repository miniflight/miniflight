"""Experimental gyro correction for the sustained VQ1 yaw-rate probe."""

import math


class YawRateFeedback:
    """Slow integral correction around the simulator's existing rate controller.

    Rates and correction limits are FRD rad/s; gain is 1/s and dt is seconds.
    A zero request remains an exact zero so handing control
    back to a NED command cannot leave a residual yaw request in the simulator.
    Integration pauses after reference steps larger than step_threshold so it
    does not fight the target's initial transient. settle_time is in seconds.
    """

    def __init__(self, gain=2.0, max_correction=.15, max_rate=1.0, settle_time=.15, step_threshold=.05):
        if not all(math.isfinite(value) and value > 0
                   for value in (gain, max_correction, max_rate, settle_time, step_threshold)):
            raise ValueError("yaw feedback parameters must be finite and positive")
        self.gain, self.max_correction, self.max_rate = gain, max_correction, max_rate
        self.settle_time, self.step_threshold = settle_time, step_threshold
        self.correction = 0.0
        self._previous_requested = None
        self._settling = 0.0

    def reset(self):
        self.correction = 0.0
        self._previous_requested = None
        self._settling = 0.0

    def update(self, requested, measured, dt):
        if not all(math.isfinite(value) for value in (requested, measured, dt)):
            raise ValueError("yaw feedback inputs must be finite")
        valid_interval = 0 < dt <= .1
        if requested == 0 or not valid_interval:
            self.reset()
        elif self._previous_requested is None or abs(requested - self._previous_requested) > self.step_threshold:
            self._settling = self.settle_time
        elif self._settling > 0:
            self._settling = max(0.0, self._settling - dt)
        else:
            # The new request has not been sent yet. Integrate error against the
            # reference that was active during the elapsed sensor interval.
            error = self._previous_requested - measured
            output = requested + self.correction
            # Allow integration at saturation only when it moves back into range.
            if ((output < self.max_rate or error < 0)
                    and (output > -self.max_rate or error > 0)):
                self.correction = max(-self.max_correction, min(self.max_correction,
                                      self.correction + self.gain * error * dt))
        self._previous_requested = requested if valid_interval else None
        return max(-self.max_rate, min(self.max_rate, requested + self.correction))
