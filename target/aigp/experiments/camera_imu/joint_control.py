"""Fly one slow arc using the joint camera/IMU state for feedback."""
from bisect import bisect_left

import numpy as np
from scipy.spatial.transform import Rotation

from miniflight import vehicle
from miniflight.position import acceleration_control, quadratic_resistance
from . import sensors as base
from .joint_estimator import JointEstimator


def reference(start, forward, speed, angle, duration, braking, elapsed):
    right = np.cross([0., 0., 1.], forward)
    t = min(elapsed, duration)
    rate = angle/duration
    theta = rate*t
    position = start+speed/rate*(np.sin(theta)*forward+(1-np.cos(theta))*right)
    direction = np.cos(theta)*forward+np.sin(theta)*right
    velocity = speed*direction
    acceleration = speed*rate*(-np.sin(theta)*forward+np.cos(theta)*right)
    if elapsed > duration:
        b = min(elapsed-duration, braking)
        position += speed*(b-.5*b*b/braking)*direction
        velocity = speed*(1-b/braking)*direction
        acceleration = -speed/braking*direction if b < braking else np.zeros(3)
    return position, velocity, acceleration


def slow_reference(start, forward, initial_speed, angle, elapsed):
    if elapsed < 1.:
        acceleration = (1.-initial_speed)*forward
        return (start+initial_speed*elapsed*forward+.5*acceleration*elapsed**2,
                initial_speed*forward+acceleration*elapsed, acceleration)
    turn_start = start+.5*(initial_speed+1.)*forward
    return reference(turn_start, forward, 1., angle, 2., 1., elapsed-1.)


class GateController(base.GateController):
    def __init__(self, calls):
        super().__init__()
        self.joint = JointEstimator(call=calls)
        self.raw_imu = []
        self.active = False
        self.settled = None
        self.last_joint_gate_time = None
        self.future_frames = []

    def _imu(self):
        remaining = []
        for sample in self.raw_imu:
            if not self.inertial.times or sample.time < self.inertial.times[0]:
                continue
            index = bisect_left(self.inertial.times, sample.time)
            if index >= len(self.inertial.times):
                remaining.append(sample)
                continue
            if self.inertial.times[index] == sample.time:
                rotation = self.inertial.rotations[index]
            else:
                rotation = self.inertial.camera_pose(sample.time)[1]
            if self.joint.integral.times and sample.time <= self.joint.integral.times[-1]:
                continue
            self.joint.imu(sample.time, sample.dt, rotation @ base.CAMERA_FROM_BODY @ sample.acceleration)
        self.raw_imu = remaining

    def observe(self, frame, stamp, position, rotation):
        if not self.joint.ready:
            super().observe(frame, stamp, position, rotation)
        if self.began is None or self.inertial.time-self.began < 5.5:
            return
        self._imu()
        gate = None
        if not self.joint.ready and self.last_image_id == frame.id:
            pose = base.vision.best_pose(self.last_corners)
            if pose is not None:
                gate = pose[2]
                self.last_joint_gate_time = stamp
        was_empty = self.joint.reference is None
        self.joint.observe(frame, rotation, gate)
        if was_empty and self.joint.reference is not None:
            self.reference_position = np.array(position)

    def update(self, telemetry, frames, race_state):
        state = vehicle.state
        assert state.motion is None and state.attitude is None
        assert all(packet.data.get_type() in base.ALLOWED for packet in telemetry)
        self.raw_imu.extend(state.imu_increments)
        if not self.active:
            return super().update(telemetry, frames, None)
        self.inertial.update(state.imu_increments)
        self._imu()
        now = self.inertial.time
        self.future_frames.extend(frames)
        ready = [f for f in self.future_frames if f.time_ns*1e-9 <= now]
        self.future_frames = [f for f in self.future_frames if f.time_ns*1e-9 > now]
        if ready:
            frame = max(ready, key=lambda f:f.time_ns)
            pose = self.inertial.camera_pose(frame.time_ns*1e-9)
            if pose is not None:
                self.joint.observe(frame, pose[1])
        return self._drive_joint()

    def drive(self, target):
        self._imu()
        if not self.active and self.joint.ready and self.joint.solutions >= 2:
            state = self.joint.predict(self.inertial.time)
            self.start = self.reference_position+self.joint.reference @ state[:3]
            velocity = self.joint.reference @ state[3:6]
            self.forward = velocity*[1, 1, 0]
            if not .8 < np.linalg.norm(self.forward) < 3.:
                raise RuntimeError('joint initialization has unusable cruising velocity')
            self.speed = float(np.clip(np.linalg.norm(self.forward), 1., 2.))
            self.forward /= np.linalg.norm(self.forward)
            self.started = self.joint.integral.at(self.inertial.time)[0]
            self.active = True
        return self._drive_joint() if self.active else super().drive(target)

    def _drive_joint(self):
        now = self.inertial.time
        age = now-self.joint.solution_time
        if age > .35:
            raise TimeoutError('joint visual estimate exceeded its 0.35s age limit')
        state = self.joint.predict(now)
        position = self.reference_position+self.joint.reference @ state[:3]
        velocity = self.joint.reference @ state[3:6]
        elapsed = self.joint.integral.at(now)[0]-self.started
        angle = np.radians(20.)*base.SETTINGS.get('joint_turn_sign', 1)
        target, wanted, feedforward = slow_reference(self.start, self.forward, self.speed, angle, elapsed)
        correction = Rotation.from_rotvec(state[6:9]).as_matrix().T
        body = self.joint.reference @ correction @ self.joint.reference.T @ self.inertial.rotation.as_rotation_matrix()
        feedback = 1.2*(target-position)+2.*(wanted-velocity)
        acceleration = feedforward+feedback
        acceleration += body @ quadratic_resistance(body.T @ velocity, self.config.body_resistance_per_m)
        phase = 'JOINT_SLOWDOWN' if elapsed < 1. else ('JOINT_TURN' if elapsed < 3. else 'JOINT_BRAKE')
        self.signal = dict(phase=phase, imu_time=now,
            position=position.tolist(), velocity=velocity.tolist(), target=target.tolist(),
            target_velocity=wanted.tolist(), target_acceleration=feedforward.tolist(),
            feedback_acceleration=feedback.tolist(), control_estimate_source='joint_camera_imu',
            motion_available=False, attitude_available=False,
            joint=dict(reference_id=self.joint.reference_id, reference_time=self.joint.reference_time,
                reference_rotation=self.joint.reference.tolist(), reference_position=self.reference_position.tolist(),
                position_camera=state[:3].tolist(), velocity_camera=state[3:6].tolist(),
                solutions=self.joint.solutions, depth_revisions=self.joint.depth_revisions,
                depth_change=round(self.joint.depth_change, 6), points=int(self.joint.tracker.alive.sum()),
                solution_time=self.joint.solution_time, age_s=age, solve_ms=self.joint.timings[-1],
                image_id=self.joint.last_id, measured_points=self.joint.tracker.points[self.joint.tracker.alive].tolist(),
                camera_rotation=self.joint.last_camera_rotation.tolist(),
                elapsed_s=float(elapsed), speed=self.speed, start_position=self.start.tolist(),
                forward=self.forward.tolist(), gate_pose_frozen_at=self.last_joint_gate_time))
        steady = elapsed >= 4. and np.linalg.norm(velocity) < .2 and np.linalg.norm(target-position) < .45
        self.settled = (self.settled or now) if steady else None
        if self.settled is not None and now-self.settled >= .25:
            self.phase = base.Phase.COMPLETE
            self.signal['phase'] = 'COMPLETE'
            raise StopIteration
        if elapsed > 6.:
            raise TimeoutError('joint turn did not settle within six seconds')
        return acceleration_control(self.config, acceleration, Rotation.from_matrix(body).as_euler('xyz'), 0.)


class Controller(base.BaseController):
    targets = ('vq1.r1',)

    def __init__(self, calls):
        self.gate = GateController(calls)
        self.motor = base.Motors(position=self.gate)

    def update(self, telemetry, frames, race_state):
        return self.motor.update(telemetry, frames, None)
