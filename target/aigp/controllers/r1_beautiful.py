"""Follow NED targets with IMU attitude registered to one native initial heading."""

from enum import Enum, auto

from common.math import Quaternion, Vector3D
from miniflight import BodyRates, vehicle
from miniflight.attitude import GyroAttitude, stationary_rotation
from miniflight.position import PositionConfig, acceleration_control, quadratic_resistance, trajectory_acceleration
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_gates import GateFollower


CONFIG = PositionConfig(hover_thrust=.266, thrust_acceleration=53.5)


class AttitudePhase(Enum):
    COLLECTING_GRAVITY = auto()
    WAITING_FOR_HEADING = auto()
    TRACKING = auto()


class ReferencePhase(Enum):
    GATE_TARGETS = auto()
    WAITING_FOR_CLOCK = auto()
    FOLLOWING = auto()


class Controller(BaseController[BodyRates]):
    targets = ("vq1.r1",)

    def __init__(self, kp=2.0, ki=0.0, kd=2.0, exit_distance=1.0, config=CONFIG, reference=None, rate_reference=None):
        self.kp = kp  # 1/s²
        self.ki = ki  # 1/s³
        self.kd = kd  # 1/s
        self.config = config
        self.follower = GateFollower(exit_distance)
        self.target = self.target_yaw = None
        self.target_velocity = Vector3D()  # NED m/s.
        self.target_acceleration = Vector3D()  # NED m/s².
        self.reference, self.reference_start = reference, None
        self.reference_phase = ReferencePhase.GATE_TARGETS if reference is None else ReferencePhase.WAITING_FOR_CLOCK
        self.rate_reference = rate_reference  # Saved FRD rad/s, sampled on the same reference clock.
        self.reference_time = self.requested_acceleration = None
        self.time = None
        self.attitude_estimate, self.calibration = None, {}
        self.attitude_phase = AttitudePhase.COLLECTING_GRAVITY
        self.integral = (0, 0, 0)  # Integral contribution, m/s²; bounded by available acceleration.

    def feedback_velocity(self, state, rotation):
        return Vector3D(*state.motion.velocity)

    def feedback_position(self, state):
        return Vector3D(*state.motion.position)

    def feedback_time(self, state, now, completed):
        motion = state.motion
        if motion is None:
            return None
        if not completed and now - motion.received_at > .3:
            raise TimeoutError("privileged pose is stale")
        return motion.time

    def update(self, telemetry, frames, race_state) -> BodyRates | None:
        if self.reference_phase is ReferencePhase.GATE_TARGETS:
            self.follower.observe_course(telemetry)
        state = vehicle.state
        if state is None or state.time is None or state.gyro is None or state.acceleration is None or race_state is None:
            return None
        now = race_state.received_at if race_state.control_time is None else race_state.control_time
        if race_state.started and now - state.received_at > .3:
            raise TimeoutError("IMU is stale")
        if self.attitude_phase is not AttitudePhase.TRACKING:
            self.calibration[state.time] = (state.received_at, state.acceleration)
            self.calibration = {t: sample for t, sample in self.calibration.items() if sample[0] >= now-1}
            if not race_state.started:
                return None
            self.attitude_phase = AttitudePhase.WAITING_FOR_HEADING
            specific_force = tuple(sum(a[i] for _, a in self.calibration.values()) / len(self.calibration) for i in range(3))
            initial_rotation = stationary_rotation(specific_force)
            if state.attitude is None:
                return None
            if now - state.attitude.received_at > .3:
                raise TimeoutError("initial heading is stale")
            self.target_yaw = state.attitude.yaw
            heading_to_ned = Quaternion.from_euler(0, 0, self.target_yaw)
            initial_rotation = heading_to_ned * initial_rotation
            self.attitude_estimate = GyroAttitude(initial_rotation, state.rate_feedback)
            self.calibration.clear()
            self.attitude_phase = AttitudePhase.TRACKING
        imu_rotation = self.attitude_estimate.update(state.rate_feedback)
        if not race_state.started:
            return None
        if self.reference_phase is ReferencePhase.GATE_TARGETS and not self.follower.has_course_for(race_state):
            return None
        stamp = self.feedback_time(state, now, race_state.completed)
        if stamp is None:
            return None
        velocity = self.feedback_velocity(state, imu_rotation)
        position = self.feedback_position(state)
        self.angles = angles = imu_rotation.to_euler()

        if self.reference_phase is ReferencePhase.GATE_TARGETS:
            index = race_state.active_gate_index
            if not 0 <= index <= len(self.follower.track):
                raise ValueError("gate index outside the published track")
            self.target = self.follower.select_target(position, race_state)
        else:
            if self.reference_phase is ReferencePhase.WAITING_FOR_CLOCK:
                self.reference_start = stamp - race_state.time_seconds
                self.reference_phase = ReferencePhase.FOLLOWING
            self.reference_time = stamp - self.reference_start
            self.target, self.target_velocity, self.target_acceleration = (Vector3D(*v) for v in self.reference(self.reference_time))
        if self.target is None:
            return None
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
