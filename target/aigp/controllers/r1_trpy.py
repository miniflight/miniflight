"""Take off with native position hold, then hold for five seconds through body rates."""

import math
from common.math import Vector3D
from miniflight import BodyRates, PositionNed
from miniflight.position import PositionConfig, acceleration_control
from target.aigp.client import RaceStatus
from target.aigp.controllers import BaseController


CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[PositionNed | BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, kp=2.0, kd=2.0):
        self.kp = kp  # 1/s²
        self.kd = kd  # 1/s
        self.privileged = {}  # Original labelled pose packets, retained between arrivals.
        self.imu = self.race = None
        self.target = self.target_yaw = None
        self.started_at = self.steady_at = self.hold_started_at = None
        self.completed = False

    def desired_acceleration(self, position: Vector3D, velocity: Vector3D,
                             target: Vector3D) -> Vector3D:
        """Position in metres and velocity in m/s produce acceleration in m/s²."""
        return self.kp * (target - position) - self.kd * velocity

    def update(self, telemetry, frames) -> PositionNed | BodyRates | None:
        for packet in telemetry:
            kind = packet.data.get_type()
            if packet.privileged and kind in ("LOCAL_POSITION_NED", "ATTITUDE"):
                self.privileged[kind] = packet
            elif kind == "HIGHRES_IMU":
                self.imu = packet
            elif isinstance(packet.decoded, RaceStatus):
                self.race = packet.decoded
        if self.imu is None or len(self.privileged) != 2 or self.race is None or not self.race.started:
            return None
        motion, orientation = (self.privileged[kind] for kind in ("LOCAL_POSITION_NED", "ATTITUDE"))
        if max(self.imu.received_at - packet.received_at for packet in (motion, orientation)) > .3:
            raise TimeoutError("privileged pose is stale")
        motion, orientation, imu = motion.data, orientation.data, self.imu.data
        position = Vector3D(motion.x, motion.y, motion.z)
        velocity = Vector3D(motion.vx, motion.vy, motion.vz)
        angles = (orientation.roll, -orientation.pitch, -orientation.yaw)  # Native wire -> FRD/NED.
        gyro = (imu.xgyro, imu.ygyro, imu.zgyro)
        if not all(math.isfinite(v) for v in (*position.v, *velocity.v, *angles, *gyro)):
            raise ValueError("nonfinite hover telemetry")
        stamp = imu.time_usec * 1e-6
        if self.target is None:
            self.target = position + Vector3D(0, 0, -3)
            self.target_yaw, self.started_at = angles[2], stamp
        distance, speed = (self.target - position).magnitude(), velocity.magnitude()
        tilt = max(abs(angles[0]), abs(angles[1]))
        if self.hold_started_at is None:
            if stamp - self.started_at > 25:
                raise TimeoutError("native takeoff did not settle")
            if distance < .15 and speed < .15 and tilt < .04 and math.hypot(*gyro) < .04:
                self.steady_at = stamp if self.steady_at is None else self.steady_at
                if stamp - self.steady_at >= .6:
                    self.hold_started_at = stamp
            else:
                self.steady_at = None
            if self.hold_started_at is None:
                return PositionNed(*self.target.v)
        if distance > 1 or speed > 2 or tilt > .5:
            raise RuntimeError("hover exceeded its motion bounds")
        if stamp >= self.hold_started_at + 5:
            self.completed = True
            raise StopIteration
        acceleration = self.desired_acceleration(position, velocity, self.target)
        return acceleration_control(CONFIG, acceleration.v, angles, self.target_yaw)
