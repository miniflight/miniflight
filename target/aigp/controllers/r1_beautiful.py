"""Position feedback with IMU tilt and native yaw, then body-rate control."""

from common.math import Quaternion, Vector3D
from miniflight import BodyRates, vehicle
from miniflight.attitude import GyroAttitude, stationary_rotation
from miniflight.position import PositionConfig, acceleration_control, quadratic_resistance, trajectory_acceleration
from target.aigp.client import TrackInfo
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import gate_target, usable_track


CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, kp=2.0, ki=0.0, kd=2.0, exit_distance=1.0, config=CONFIG, reference=None, rate_reference=None):
        self.kp = kp  # 1/s²
        self.ki = ki  # 1/s³
        self.kd = kd  # 1/s
        self.exit_distance, self.config = exit_distance, config
        self.track = self.gate = None
        self.target = self.target_yaw = None
        self.target_velocity = Vector3D()  # NED m/s.
        self.target_acceleration = Vector3D()  # NED m/s².
        self.reference, self.reference_start = reference, None
        self.rate_reference = rate_reference  # Saved FRD rad/s, sampled on the same reference clock.
        self.reference_time = self.requested_acceleration = None
        self.time = None
        self.tilt, self.calibration = None, {}
        self.integral = (0, 0, 0)  # Integral contribution, m/s²; bounded by available acceleration.

    def update(self, telemetry, frames, race_state) -> BodyRates | None:
        for packet in telemetry:
            if self.reference is None and isinstance(packet.decoded, TrackInfo) and usable_track(packet.decoded.gates):
                self.track = packet.decoded.gates
        state = vehicle.state
        if state is None or state.time is None or state.gyro is None or state.acceleration is None or race_state is None:
            return None
        now = race_state.received_at if race_state.control_time is None else race_state.control_time
        if race_state.started and now - state.received_at > .3:
            raise TimeoutError("IMU is stale")
        if self.tilt is None:
            self.calibration[state.time] = (state.received_at, state.acceleration)
            self.calibration = {t: sample for t, sample in self.calibration.items() if sample[0] >= now-1}
            if not race_state.started:
                return None
            specific_force = tuple(sum(a[i] for _, a in self.calibration.values()) / len(self.calibration) for i in range(3))
            self.tilt = GyroAttitude(stationary_rotation(specific_force), state)
            self.calibration.clear()
        imu_rotation = self.tilt.update(state)
        if state.motion is None or state.attitude is None or not race_state.started:
            return None
        if self.reference is None and (self.track is None or len(race_state.gate_centers) != len(self.track)):
            return None
        motion, orientation = state.motion, state.attitude
        if not race_state.completed and max(now - sample.received_at for sample in (motion, orientation)) > .3:
            raise TimeoutError("privileged pose is stale")
        position, velocity = Vector3D(*motion.position), Vector3D(*motion.velocity)
        roll, pitch, _ = imu_rotation.to_euler()
        self.angles = angles = (roll, pitch, orientation.yaw)

        if self.reference is None:
            index = race_state.active_gate_index
            if not 0 <= index <= len(self.track):
                raise ValueError("gate index outside the published track")
            if index < len(self.track) and (index, race_state.gate_centers[index]) != self.gate:
                self.target = gate_target(position, index, race_state.gate_centers, self.exit_distance)
                self.gate = (index, race_state.gate_centers[index])
        else:
            if self.reference_start is None:
                self.reference_start = motion.time - race_state.time_seconds
            self.reference_time = motion.time - self.reference_start  # Pose seconds since native GO.
            self.target, self.target_velocity, self.target_acceleration = (Vector3D(*v) for v in self.reference(self.reference_time))
        if self.target is None:
            return None
        if self.target_yaw is None:
            self.target_yaw = angles[2]

        dt = 0.0 if self.time is None else max(0.0, race_state.time_seconds - self.time)
        self.time = race_state.time_seconds
        acceleration_request, integral = trajectory_acceleration(
            position.v, velocity.v, (self.target.v, self.target_velocity.v, self.target_acceleration.v), self.integral, dt,
            kp=self.kp, ki=self.ki, kd=self.kd, integral_limit=self.config.max_acceleration)
        drag_compensation = quadratic_resistance(velocity.v, self.config.resistance_per_m)
        if self.config.body_resistance_per_m is not None:
            body_to_ned = Quaternion.from_euler(*angles)
            body_velocity = body_to_ned.conjugate().rotate(velocity)
            body_drag_compensation = Vector3D(*quadratic_resistance(body_velocity.v, self.config.body_resistance_per_m))
            drag_compensation = body_to_ned.rotate(body_drag_compensation).v
        acceleration_request = tuple(a + d for a, d in zip(acceleration_request, drag_compensation, strict=True))
        self.integral = integral
        self.requested_acceleration = acceleration_request
        feedforward = (0, 0, 0) if self.rate_reference is None else self.rate_reference(self.reference_time)
        return acceleration_control(self.config, acceleration_request, angles, self.target_yaw, rate_feedforward=feedforward)
