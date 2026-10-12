"""Brief collective pulses during our position, attitude, and rate feedback."""

from dataclasses import replace
import math

from target.aigp.controllers.r1_beautiful import Controller as Position


def hold(time):
    return (0, 0, -1), (0, 0, 0), (0, 0, 0)


class CollectivePulse(Position):
    def __init__(self, pulses):
        super().__init__(reference=hold)
        self.pulses = tuple(pulses)  # (collective fraction, duration in source seconds).
        if not self.pulses or any(not 0 < u <= .8 or not 0 < dt <= .2 for u, dt in self.pulses):
            raise ValueError("provide fractions in (0, .8] and durations in (0, .2]")
        self.index, self.phase, self.done = 0, "settle", False
        self.wait_until, self.steady_at, self.pulse_at = 6, None, None

    def update(self, telemetry, frames, race_state):
        request = super().update(telemetry, frames, race_state)
        if request is None:
            return None
        pose, angles = (self.privileged[k].data for k in ("LOCAL_POSITION_NED", "ATTITUDE"))
        distance, speed = math.dist((pose.x, pose.y, pose.z), (0, 0, -1)), math.hypot(pose.vx, pose.vy, pose.vz)
        tilt = max(abs(angles.roll), abs(angles.pitch))
        now = self.reference_time
        if distance > 3 or speed > 3 or tilt > .4:
            raise RuntimeError("collective pulse exceeded the measured motion envelope")
        if self.phase == "pulse":
            u, duration = self.pulses[self.index]
            if now - self.pulse_at < duration:
                return replace(request, thrust=u)
            self.index += 1
            self.phase, self.wait_until, self.steady_at = "settle", now + 3, None
        if now > self.wait_until + 20:
            raise TimeoutError("collective pulse did not settle within 20 source seconds")
        if now >= self.wait_until and distance < .1 and speed < .1 and tilt < .04:
            if self.steady_at is None:
                self.steady_at = now
            if now - self.steady_at >= .6:
                if self.index == len(self.pulses):
                    self.done = True
                    raise StopIteration
                self.phase, self.pulse_at = "pulse", now
                return replace(request, thrust=self.pulses[self.index][0])
        else:
            self.steady_at = None
        return request
