"""One aperture, with an IMU frame rooted at the stationary shifted start."""

from bisect import bisect_left, bisect_right
from dataclasses import replace
from enum import Enum, auto
import json
import mmap
from pathlib import Path
import struct

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from miniflight import vehicle
from miniflight.attitude import gyro_step, stationary_rotation
from miniflight.position import PositionConfig, acceleration_control, quadratic_resistance
from miniflight.state import Frame
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_motors import Controller as Motors
from target.aigp.experiments.producer_imu import ProducerImuClient


from . import vision
from target.aigp.native import BASE


ALLOWED = frozenset(("HEARTBEAT", "COMMAND_ACK", "HIGHRES_IMU"))
CAMERA_FROM_BODY = vision.CAMERA_FROM_BODY
SETTINGS = dict(speed_m_s=4., final_distance_m=2.5, beyond_m=1.5, stop_beyond_m=1.,
                lock_frames=3, maximum_blind_s=1.5, flight_timeout_s=12., shift_right_m=-.5,
                acquisition_close_gaps=True, acquisition_largest_rim=True)


class CameraClock:
    """Read capture and timestamp boundaries, without reading rendered poses."""

    def __init__(self, path):
        with Path(path).open("r+b") as source:
            self.mapping = mmap.mmap(source.fileno(), 0)
        self.index = 0
        self.captures, self.stamps = {}, {}
        self.latest = None

    def close(self):
        self.mapping.close()

    def read(self):
        count = struct.unpack_from("<Q", self.mapping)[0]
        if count >= (len(self.mapping)-4096)//512:
            raise RuntimeError("camera clock probe exhausted")
        output = []
        while self.index < count:
            d = struct.unpack_from("<16Q", self.mapping, 4096+self.index*512)
            if not d[15]:
                break
            self.index += 1
            kind, identity = d[0], d[11]
            if kind == 1:
                origin = struct.unpack("<d", struct.pack("<Q", d[5]))[0]
                scale = struct.unpack("<d", struct.pack("<Q", d[6]))[0]
                self.captures[identity] = [d[4], None, origin, scale]
            elif kind == 8 and identity in self.captures:
                self.captures[identity][1] = d[2]
            elif kind == 2:
                self.latest = identity
            elif kind == 5 and self.latest in self.captures:
                low, high, origin, scale = self.captures[self.latest]
                if high is not None:
                    self.stamps[d[9]] = [low*scale+16777216-origin, high*scale+16777216-origin]
            elif kind == 4 and d[9] in self.stamps:
                output.append([d[10], d[9], *self.stamps[d[9]]])
        return output


class SensorClient(ProducerImuClient):
    def __init__(self, calls, camera_clock=None, **kwargs):
        super().__init__(calls, **kwargs)
        self.camera_clock = camera_clock
        self.capture_times = {}
        self.filtered = 0

    def _receive(self, message, now):
        if message.get_type() not in ALLOWED:
            return None
        return super()._receive(message, now)

    def poll(self, timeout=0.):
        packets = super().poll(timeout)
        timing = self.calls("camera_clock", {}, self.camera_clock)
        self.capture_times.update({(frame, stamp): (low+high)/2 for frame, stamp, low, high in timing})
        output = []
        for packet in packets:
            if not isinstance(packet.data, bytes) and packet.data.get_type() not in ALLOWED:
                self.filtered += 1
                continue
            if isinstance(packet.decoded, Frame):
                key = packet.decoded.id, packet.decoded.time_ns
                value = self.capture_times.get(key)
                frame = None if value is None else replace(packet.decoded, time_ns=round(value*1e9))
                packet = replace(packet, decoded=frame)
            output.append(packet)
        return tuple(output)

    def read(self, timeout=1.):
        state = super().read(timeout)
        original = state.frame
        if original is not None:
            value = self.capture_times.get((original.id, original.time_ns))
            state = replace(state, frame=None if value is None else replace(original, time_ns=round(value*1e9)))
        assert state.motion is None and state.attitude is None and self.gates is None
        self.state = state
        return state


class Phase(Enum):
    CALIBRATING = auto()
    SHIFTING = auto()
    ACQUIRING = auto()
    TRACKING = auto()
    FINAL_IMU = auto()
    COMPLETE = auto()


class Inertial:
    def __init__(self):
        self.seed = []
        self.rotation = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.raw_position, self.raw_velocity = np.zeros(3), np.zeros(3)
        self.times, self.positions, self.rotations, self.velocities = [], [], [], []

    def update(self, samples):
        for sample in samples:
            if self.rotation is None:
                self.seed.append(sample)
                if sample.time-self.seed[0].time < .5:
                    continue
                force = np.array([s.acceleration for s in self.seed])
                if np.max(np.linalg.norm([s.gyro for s in self.seed], axis=1)) > .05 or np.max(force.std(axis=0)) > .2:
                    raise RuntimeError("stationary IMU calibration failed")
                self.rotation = stationary_rotation(force.mean(axis=0))
            elif sample.time > self.time:
                gyro = .5*(np.array(self.gyro)+sample.gyro)
                self.rotation = gyro_step(self.rotation, gyro, sample.dt)
                acceleration = self.rotation.as_rotation_matrix() @ sample.acceleration + [0, 0, 9.81]
                velocity = self.velocity + acceleration*sample.dt
                self.position = self.position + .5*(self.velocity+velocity)*sample.dt
                self.velocity = velocity
                raw_velocity = self.raw_velocity + acceleration*sample.dt
                self.raw_position += .5*(self.raw_velocity+raw_velocity)*sample.dt
                self.raw_velocity = raw_velocity
            else:
                continue
            self.time, self.gyro = sample.time, sample.gyro
            self.times.append(sample.time)
            self.positions.append(self.position.copy())
            self.velocities.append(self.velocity.copy())
            self.rotations.append(self.rotation.as_rotation_matrix() @ CAMERA_FROM_BODY.T)

    def velocity_at(self, stamp):
        if not self.times[0] <= stamp <= self.times[-1]:
            raise ValueError("velocity measurement is outside the IMU history")
        return np.array([np.interp(stamp, self.times, column) for column in np.asarray(self.velocities).T])

    def correct_velocity(self, stamp, measured, gain):
        delta = gain*(np.asarray(measured)-self.velocity_at(stamp))
        for i in range(bisect_left(self.times, stamp), len(self.times)):
            self.velocities[i] += delta
            self.positions[i] += delta*(self.times[i]-stamp)
        self.position, self.velocity = self.positions[-1].copy(), self.velocities[-1].copy()
        return delta

    def correct_position(self, stamp, measured, gain):
        pose = self.camera_pose(stamp)
        if pose is None:
            raise ValueError("position measurement is outside the IMU history")
        delta = gain*(np.asarray(measured)-pose[0])
        for i in range(bisect_left(self.times, stamp), len(self.times)):
            self.positions[i] += delta
        self.position = self.positions[-1].copy()
        return delta

    def camera_pose(self, stamp):
        index = bisect_right(self.times, stamp)-1
        if index < 0 or stamp > self.times[-1]:
            return None
        if index+1 == len(self.times):
            return self.positions[index], self.rotations[index]
        times = self.times[index:index+2]
        fraction = (stamp-times[0])/(times[1]-times[0])
        position = self.positions[index]*(1-fraction)+self.positions[index+1]*fraction
        rotation = Slerp(times, Rotation.from_matrix(self.rotations[index:index+2]))(stamp).as_matrix()
        return position, rotation


class GateController(BaseController):
    targets = ("vq1.r1",)

    def __init__(self):
        config = json.loads((BASE.parents[1]/"test/fixtures/vq1_r1_sub10.config.json").read_text())
        config.update(max_speed=SETTINGS["speed_m_s"], max_acceleration=3., max_horizontal_acceleration=3., max_tilt=.45)
        self.config = PositionConfig(**config)
        self.inertial = Inertial()
        self.phase = Phase.CALIBRATING
        self.previous = None
        self.track = -1
        self.locked = None
        self.count = 0
        self.centers = []
        self.anchor = self.direction = None
        self.pending = []
        self.used_frames = []
        self.last_image_time = self.last_image_id = None
        self.last_corners = self.last_gate_corners = None
        self.final_at = self.began = None
        self.setup_started = self.setup_steady = self.acquire_started = None
        self.shift_displacement = None
        self.signal = {}

    def drive(self, target):
        p, v = self.inertial.position, self.inertial.velocity
        desired_velocity = self.config.position_gain*(target-p)
        desired_velocity *= min(1., self.config.max_speed/max(np.linalg.norm(desired_velocity), 1e-9))
        acceleration = self.config.velocity_gain*(desired_velocity-v)
        body = self.inertial.rotation.as_rotation_matrix()
        body_drag = quadratic_resistance(body.T @ v, self.config.body_resistance_per_m)
        acceleration += body @ body_drag
        return acceleration_control(self.config, acceleration, self.inertial.rotation.to_euler(), 0.)

    def observe(self, frame, stamp, position, rotation):
        corners = None
        if self.previous is not None:
            guess = vision.warped(self.previous, position, rotation, "rotation_and_motion")
            if guess is not None:
                corners = vision.refine(frame.bgr, guess)
        if corners is None and self.locked is None:
            corners = vision.nearest_aperture(frame.bgr, close_gaps=SETTINGS["acquisition_close_gaps"],
                                             largest_rim=SETTINGS["acquisition_largest_rim"])
            if corners is not None:
                self.track += 1
                self.count, self.centers = 0, []
        pose = vision.best_pose(corners)
        if pose is None:
            self.previous = None if self.locked is None else self.previous
            return
        center = position + rotation @ pose[2]
        if self.locked is not None:
            limit = max(.5, .05*np.linalg.norm(self.anchor-position))
            if np.linalg.norm(center-self.anchor) > limit:
                return
        self.previous = (position, rotation, corners, pose)
        self.count += 1
        self.centers.append(center)
        self.centers = self.centers[-5:]
        self.anchor = np.median(self.centers, axis=0)
        self.last_image_time, self.last_image_id = stamp, frame.id
        self.last_corners = corners.copy()
        self.last_gate_corners = pose[1] @ rotation.T + position
        self.used_frames.append(frame.id)
        if self.locked is None and self.count >= SETTINGS["lock_frames"]:
            self.locked = self.track
            self.direction = self.anchor*np.array([1, 1, 0])
            self.direction /= np.linalg.norm(self.direction)
            self.began = self.inertial.time
            self.phase = Phase.TRACKING

    def update(self, telemetry, frames, race_state):
        state = vehicle.state
        assert state.motion is None and state.attitude is None
        assert all(packet.data.get_type() in ALLOWED for packet in telemetry)
        self.inertial.update(state.imu_increments)
        if self.inertial.rotation is None:
            return None
        if self.phase is Phase.CALIBRATING:
            self.phase = Phase.SHIFTING
            self.setup_started = self.inertial.time
        if state.time-self.inertial.time > .1:
            raise TimeoutError("producer IMU stopped before the wire stream")
        now = self.inertial.time
        if self.phase is Phase.SHIFTING:
            target = np.array([0., SETTINGS["shift_right_m"], -1.])
            distance = np.linalg.norm(target-self.inertial.position)
            steady = distance < .04 and np.linalg.norm(self.inertial.velocity) < .08
            self.setup_steady = (self.setup_steady or now) if steady else None
            if now-self.setup_started > 10.:
                raise TimeoutError("IMU shift and settle exceeded ten seconds")
            if self.setup_steady is not None and now-self.setup_steady >= .4:
                self.shift_displacement = self.inertial.position.copy()
                self.inertial.position = np.zeros(3)
                self.inertial.times = [now]
                self.inertial.positions = [self.inertial.position.copy()]
                self.inertial.velocities = [self.inertial.velocity.copy()]
                self.inertial.raw_position = np.zeros(3)
                self.inertial.rotations = [self.inertial.rotation.as_rotation_matrix() @ CAMERA_FROM_BODY.T]
                self.phase = Phase.ACQUIRING
                self.acquire_started = now
                self.pending = []
            else:
                self.signal = dict(phase=self.phase.name, imu_time=now, position=self.inertial.position.tolist(),
                    velocity=self.inertial.velocity.tolist(), target=target.tolist(),
                    motion_available=False, attitude_available=False)
                return self.drive(target)
        self.pending.extend(frames)
        for frame in self.pending:
            stamp = frame.time_ns*1e-9
            pose = self.inertial.camera_pose(stamp)
            if pose is not None and self.phase is not Phase.FINAL_IMU:
                self.observe(frame, stamp, *pose)
        self.pending = [frame for frame in self.pending if frame.time_ns*1e-9 > now]
        if self.locked is None:
            if now-self.acquire_started > 5.:
                raise TimeoutError("no confirmed opening in the largest visible gate rim")
            self.signal = dict(phase=self.phase.name, imu_time=now, position=self.inertial.position.tolist(),
                velocity=self.inertial.velocity.tolist(), target=[0., 0., 0.],
                shift_displacement=self.shift_displacement.tolist(), motion_available=False, attitude_available=False)
            return self.drive(np.zeros(3))
        p, v = self.inertial.position, self.inertial.velocity
        remaining = float((self.anchor-p) @ self.direction)
        if self.phase is Phase.TRACKING and remaining <= SETTINGS["final_distance_m"]:
            self.phase, self.final_at = Phase.FINAL_IMU, now
        if remaining < -SETTINGS["stop_beyond_m"]:
            self.phase = Phase.COMPLETE
            raise StopIteration
        if now-self.began > SETTINGS["flight_timeout_s"]:
            raise TimeoutError("one-gate approach exceeded its IMU time budget")
        if self.phase is Phase.FINAL_IMU and now-self.last_image_time > SETTINGS["maximum_blind_s"]:
            raise TimeoutError("final gate estimate exceeded its blind interval")
        target = self.anchor + SETTINGS["beyond_m"]*self.direction
        self.signal = dict(phase=self.phase.name, imu_time=now, position=p.tolist(), velocity=v.tolist(),
            gate_center=self.anchor.tolist(), remaining_m=remaining, target=target.tolist(),
            image_time=self.last_image_time, image_id=self.last_image_id, image_age_s=now-self.last_image_time,
            accepted_images=len(self.used_frames), track=self.locked, final_at=self.final_at,
            image_corners=self.last_corners.tolist(), gate_corners=self.last_gate_corners.tolist(),
            camera_orientation=self.inertial.rotations[-1].tolist(),
            shift_displacement=self.shift_displacement.tolist(),
            motion_available=state.motion is not None, attitude_available=state.attitude is not None)
        return self.drive(target)
