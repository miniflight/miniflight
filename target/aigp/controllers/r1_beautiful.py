"""Position feedback, attitude feedback, then native body-rate control."""

import math
from common.math import Vector3D
from miniflight import BodyRates
from miniflight.position import PositionConfig, acceleration_control
from target.aigp.client import TrackInfo
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import gate_target, usable_track


CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, kp=2.0, ki=0.0, kd=2.0, exit_distance=1.0, config=CONFIG):
        self.kp = kp  # 1/s²
        self.ki = ki  # 1/s³
        self.kd = kd  # 1/s
        self.exit_distance, self.config = exit_distance, config
        self.privileged = {}  # Original labelled pose packets, retained between arrivals.
        self.imu = None
        self.track = self.gate = None
        self.target = self.target_yaw = None
        self.time = None
        self.integral = Vector3D()  # Integral contribution, m/s²; bounded by available acceleration.

    def desired_acceleration(self, position: Vector3D, velocity: Vector3D,
                             target: Vector3D, dt: float) -> Vector3D:
        """Position in metres and velocity in m/s produce acceleration in m/s²."""
        error = target - position
        integral = self.integral + self.ki * dt * error
        limit = self.config.max_acceleration
        self.integral = Vector3D(*(max(-limit, min(limit, v)) for v in integral.v))
        return self.kp * error + self.integral - self.kd * velocity

    def update(self, telemetry, frames, race_state) -> BodyRates | None:
        for packet in telemetry:
            kind = packet.data.get_type()
            if packet.privileged and kind in ("LOCAL_POSITION_NED", "ATTITUDE"):
                self.privileged[kind] = packet
            elif packet.privileged and isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
            elif kind == "HIGHRES_IMU":
                self.imu = packet
        if self.imu is None or len(self.privileged) != 2 or race_state is None or not race_state.started or self.track is None:
            return None
        motion, orientation = (self.privileged[kind] for kind in ("LOCAL_POSITION_NED", "ATTITUDE"))
        if max(self.imu.received_at - packet.received_at for packet in (motion, orientation)) > .3:
            raise TimeoutError("privileged pose is stale")
        motion, orientation, imu = motion.data, orientation.data, self.imu.data
        position = Vector3D(motion.x, motion.y, motion.z)
        velocity = Vector3D(motion.vx, motion.vy, motion.vz)
        angles = (orientation.roll, -orientation.pitch, -orientation.yaw)  # Native wire -> FRD/NED.
        if not all(math.isfinite(v) for v in (*position.v, *velocity.v, *angles)):
            raise ValueError("nonfinite control telemetry")
        stamp = imu.time_usec * 1e-6
        dt = 0.0 if self.time is None else max(0.0, stamp - self.time)
        self.time = stamp
        index = race_state.active_gate_index
        if not 0 <= index <= len(self.track):
            raise ValueError("gate index outside the published track")
        if index < len(self.track) and self.track[index] != self.gate:
            self.target = Vector3D(*gate_target(position.v, index, self.track, self.exit_distance))
            self.gate = self.track[index]
        if self.target is None:
            return None
        if self.target_yaw is None:
            self.target_yaw = angles[2]
        acceleration = self.desired_acceleration(position, velocity, self.target, dt)
        return acceleration_control(self.config, acceleration.v, angles, self.target_yaw)
