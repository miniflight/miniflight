"""R1 TRPY controller, starting with position feedback in NED."""

from common.math import Vector3D
from target.aigp.controllers import BaseController


class Controller(BaseController):
    targets = ("vq1.r1",)

    def __init__(self, track=None, kp=2.0, kd=2.0):
        self.kp = kp  # 1/s²
        self.kd = kd  # 1/s

    def desired_acceleration(self, position: Vector3D, velocity: Vector3D,
                             target: Vector3D) -> Vector3D:
        """Position in metres and velocity in m/s produce acceleration in m/s²."""
        return self.kp * (target - position) - self.kd * velocity

    def update(self, telemetry, frames):
        raise NotImplementedError("acceleration to TRPY conversion is not implemented yet")
