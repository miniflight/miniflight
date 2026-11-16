"""One aperture, with an IMU frame rooted at the stationary shifted start."""

from bisect import bisect_left, bisect_right
from dataclasses import replace
from enum import Enum, auto
import json
import mmap
from pathlib import Path
import struct

import numpy as np
import cv2
from scipy.spatial.transform import Rotation, Slerp
from common.math import Quaternion

from miniflight import vehicle
from miniflight.attitude import gyro_step, stationary_rotation
from miniflight.position import PositionConfig, acceleration_control, quadratic_resistance
from miniflight.state import Frame
from miniflight.profiling import timed
from target.aigp.controllers import BaseController
from target.aigp.controllers.r1_motors import Controller as Motors
from target.aigp.experiments.producer_imu import ProducerImuClient


from . import vision
from .motion import GateMotion,transverse_velocity
from .gravity import GateGravity
from .partial import RimTemplate,outer_top,relative_from_top,predicted_aperture,same_red_component
from target.aigp.native import BASE


ALLOWED = frozenset(("HEARTBEAT", "COMMAND_ACK", "HIGHRES_IMU"))
CAMERA_FROM_BODY = vision.CAMERA_FROM_BODY
SETTINGS = dict(speed_m_s=4., final_distance_m=2.5, beyond_m=1.5, stop_beyond_m=1.,
                lock_frames=3, maximum_blind_s=1.5, flight_timeout_s=12., shift_right_m=-.5,
                acquisition_close_gaps=True, acquisition_largest_rim=True)


def velocity_target(delta,gain,speed,vertical_speed=None):
    desired=gain*np.asarray(delta)
    if vertical_speed is None:
        return desired*min(1.,speed/max(np.linalg.norm(desired),1e-9))
    desired[:2]*=min(1.,speed/max(np.linalg.norm(desired[:2]),1e-9))
    desired[2]=np.clip(desired[2],-vertical_speed,vertical_speed)
    return desired


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
        self.force_model=None
        self.prediction_bias=np.zeros(3)
        self.seed = []
        self.rotation = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.raw_position, self.raw_velocity = np.zeros(3), np.zeros(3)
        self.times, self.positions, self.rotations, self.velocities = [], [], [], []

    @timed('prediction')
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
                previous_body=self.rotation.as_rotation_matrix()
                self.rotation = gyro_step(self.rotation, gyro, sample.dt)
                body=self.rotation.as_rotation_matrix()
                if self.force_model is None:
                    acceleration = body @ sample.acceleration + [0, 0, 9.81]
                else:
                    acceleration=.5*(previous_body @ self.force_model(previous_body,self.velocity)
                        +body @ self.force_model(body,self.velocity))+[0.,0.,9.81]+self.prediction_bias
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

    @timed('velocity_correction')
    def correct_velocity(self, stamp, measured, gain):
        innovation=np.asarray(measured)-self.velocity_at(stamp)
        delta = gain*innovation
        if self.force_model is not None:
            self.prediction_bias=np.clip(self.prediction_bias+.3*delta,-3.,3.)
        for i in range(bisect_left(self.times, stamp), len(self.times)):
            self.velocities[i] += delta
            self.positions[i] += delta*(self.times[i]-stamp)
        self.position, self.velocity = self.positions[-1].copy(), self.velocities[-1].copy()
        return delta

    @timed('position_correction')
    def correct_position(self, stamp, measured, gain):
        pose = self.camera_pose(stamp)
        if pose is None:
            raise ValueError("position measurement is outside the IMU history")
        delta = gain*(np.asarray(measured)-pose[0])
        for i in range(bisect_left(self.times, stamp), len(self.times)):
            self.positions[i] += delta
        self.position = self.positions[-1].copy()
        return delta

    @timed('attitude_correction')
    def correct_orientation(self,stamp,correction):
        matrix=correction.as_matrix()
        for i in range(bisect_left(self.times,stamp),len(self.times)):
            self.rotations[i]=matrix @ self.rotations[i]
        x,y,z,w=correction.as_quat()
        self.rotation=Quaternion(w,x,y,z)*self.rotation

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
        supplied=SETTINGS.get('position_config')
        config=dict(supplied) if supplied is not None else json.loads((BASE.parents[1]/"test/fixtures/vq1_r1_sub10.config.json").read_text())
        config.update(max_speed=SETTINGS["speed_m_s"], max_acceleration=3., max_horizontal_acceleration=3., max_tilt=.45)
        self.config = PositionConfig(**config)
        self.inertial = Inertial()
        self.timing={};self.inertial.timing=self.timing
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
        self.camera_motion = GateMotion(SETTINGS.get("camera_window_s", .25))
        self.camera_observation = None
        self.gravity=GateGravity()
        self.gravity_observation=None
        self.rim_template=RimTemplate();self.partial_motion=GateMotion(.5);self.partial_observation=None
        self.full_image_time=None;self.partial_retargets=0
        self.association_counts=dict(full_updates=0,tracked_full_updates=0,partial_updates=0,
            same_component_full_preferred=0,nearer_partial_proposals=0,farther_full_rejected=0)
        self.camera_fusion_started = False
        self.last_relative_gate=None
        self.last_relative_position=None

    def gate_relative(self):
        return self.last_relative_gate-(self.inertial.position-self.last_relative_position)

    def latest_camera_observation(self):
        full=self.camera_observation;partial=self.partial_observation
        if partial is not None and partial['gate_number']==getattr(self,'gate_number',1):
            if full is None or partial['capture_time']>full['capture_time']:return partial
        return full

    def measurement_time(self):
        observed=self.latest_camera_observation()
        stamp=-float('inf') if self.last_image_time is None else self.last_image_time
        return stamp if observed is None else max(stamp,observed['capture_time'])

    def record_visual(self,frame,stamp,kind,points,**quality):
        calls=getattr(self,'calls',None)
        if calls is not None and calls.record is not None:
            calls.record(event='visual_measurement',image_id=frame.id,capture_time=stamp,kind=kind,
                points=np.asarray(points).tolist(),context_gate=getattr(self,'gate_number',1),quality=quality)

    @timed('partial_measurement')
    def observe_partial(self,frame,stamp,rotation,position,full_relative=None,full_corners=None):
        if not SETTINGS.get('partial_gate') or self.locked is None:return False
        top=outer_top(frame.bgr)
        if top is None:return False
        template=self.rim_template.model()
        self.record_visual(frame,stamp,'outer_top',top[0],edge_residual_px=top[1],template=template)
        if template is None:return False
        if full_corners is not None and same_red_component(frame.bgr,full_corners,top[0]):
            self.association_counts['same_component_full_preferred']+=1
            return False
        relative=relative_from_top(top[0],rotation,template['width_m'],template['center_to_top_m'])
        if relative is None or relative @ self.direction<1.:return False
        if full_relative is not None and relative @ self.direction>=full_relative @ self.direction-5.:return False
        prior=self.gate_relative() @ self.direction
        if full_relative is not None and relative @ self.direction<prior-5. and hasattr(self,'nearer_gate'):
            self.association_counts['nearer_partial_proposals']+=1
            predicted=predicted_aperture(top[0],rotation,template['width_m'],template['center_to_top_m'])
            if predicted is None:return False
            track=self.locked
            self.nearer_gate(frame,stamp,position,rotation,*predicted,position+relative)
            if self.locked==track:return True
            self.last_image_id=None;self.full_image_time=None;self.partial_retargets+=1
            self.partial_motion.observations.clear();self.partial_observation=None
            prior=self.gate_relative() @ self.direction
        if abs(relative @ self.direction-prior)>max(5.,.5*abs(prior)):return False
        raw_relative=relative.copy()
        distance=prior+.05*(relative @ self.direction-prior)
        ray=rotation @ (vision.K_INV @ np.r_[np.mean(top[0],axis=0),1.])
        forward_ray=ray @ self.direction
        if forward_ray<=.1:return False
        relative_top=ray/forward_ray*distance
        relative=relative_top+[0.,0.,template['center_to_top_m']]
        gate=getattr(self,'gate_number',1);old=self.partial_observation
        if old is None or old['gate_number']!=gate or stamp-old['capture_time']>.15:self.partial_motion.observations.clear()
        self.partial_motion.observe(stamp,-relative_top)
        bearing=self.partial_motion.bearing_motion(self.direction);velocity=None
        delta_v=np.zeros(3)
        if bearing is not None:
            speed=float(self.inertial.velocity_at(bearing['time']) @ self.direction)
            measured_v=transverse_velocity(bearing,self.direction,speed)
            velocity=dict(time=bearing['time'],velocity=measured_v.tolist(),window_s=bearing['window_s'],
                observations=bearing['observations'],residual_m=bearing['range_m']*bearing['residual_rad'],radial_component='model carried')
            if np.linalg.norm(measured_v)<=8.:delta_v=self.inertial.correct_velocity(velocity['time'],measured_v,.15)
        delta_p=self.inertial.correct_position(stamp,self.anchor-relative,.2)
        self.last_relative_gate=relative;self.last_relative_position=self.inertial.camera_pose(stamp)[0].copy()
        self.partial_observation=dict(gate_number=gate,image_id=frame.id,capture_time=stamp,position_relative_gate=(-relative).tolist(),
            top_corners=top[0].tolist(),edge_residual_px=top[1],template=template,velocity=velocity,
            raw_position_relative_gate=(-raw_relative).tolist(),radial_position_gain=.05,bearing_motion=bearing,
            position_correction=delta_p.tolist(),velocity_correction=delta_v.tolist())
        self.association_counts['partial_updates']+=1
        return True

    def gate_remaining(self):
        if SETTINGS.get('continuous_course') and self.last_relative_gate is not None:
            return float(self.gate_relative() @ self.direction)
        return float((self.anchor-self.inertial.position) @ self.direction)

    def final_approach_ready(self):
        return True

    @timed('position_feedback')
    def drive(self, target):
        p, v = self.inertial.position, self.inertial.velocity
        desired_velocity = velocity_target(target-p,self.config.position_gain,self.config.max_speed,
            SETTINGS.get('vertical_speed_limit_m_s') if SETTINGS.get('continuous_course') else None)
        acceleration = self.config.velocity_gain*(desired_velocity-v)
        body = self.inertial.rotation.as_rotation_matrix()
        body_drag = quadratic_resistance(body.T @ v, self.config.body_resistance_per_m)
        acceleration += body @ body_drag
        return acceleration_control(self.config, acceleration, self.inertial.rotation.to_euler(), 0.)

    @timed('gate_measurement')
    def observe(self, frame, stamp, position, rotation):
        if SETTINGS.get("camera_correction") and self.camera_motion.observations and stamp <= self.camera_motion.observations[-1][0]:
            return
        corners = None
        guess=None
        if self.previous is not None:
            guess = vision.warped(self.previous, position, rotation, 'rotation' if SETTINGS.get('continuous_course') else "rotation_and_motion")
            if guess is not None:
                strict=SETTINGS.get('continuous_course',False)
                corners = vision.refine(frame.bgr, guess,minimum_supported=4 if strict else 2,red_edges=strict)
                if strict and corners is not None:
                    check=vision.best_pose(corners)
                    if check is None or check[0]>1.:corners=None
                    elif cv2.contourArea(corners.astype(np.float32))<.5*cv2.contourArea(np.asarray(guess,np.float32)):
                        corners=None
                if SETTINGS.get('continuous_course') and corners is not None and (np.any(corners<1) or np.any(corners>=[639,359])):
                    corners=None
        tracked=corners is not None
        if corners is None and (self.locked is None or SETTINGS.get('continuous_course')):
            corners = vision.nearest_aperture(frame.bgr, close_gaps=SETTINGS["acquisition_close_gaps"],
                                             largest_rim=SETTINGS["acquisition_largest_rim"],strict=SETTINGS.get('continuous_course',False))
            if corners is not None:
                self.track += 1
                self.count, self.centers = 0, []
        pose = vision.best_pose(corners)
        if pose is not None and SETTINGS.get('continuous_course') and pose[0]>1.:pose=None
        if pose is not None:self.record_visual(frame,stamp,'aperture',corners,reprojection_px=pose[0],tracked=tracked)
        full_relative=None if pose is None else rotation @ pose[2]
        if self.observe_partial(frame,stamp,rotation,position,full_relative,corners if pose is not None else None):return
        if pose is not None and not tracked and SETTINGS.get('partial_gate') and self.locked is not None and (guess is None or vision.iou(corners,guess)<.25):
            predicted_range=self.gate_relative() @ self.direction
            if full_relative @ self.direction>predicted_range+max(5.,.5*abs(predicted_range)):
                self.association_counts['farther_full_rejected']+=1
                pose=None
        if pose is None:
            self.previous = None if self.locked is None else self.previous
            return
        if SETTINGS.get('continuous_course') and pose[0]>1.:
            return
        if SETTINGS.get('continuous_course') and self.locked is not None and guess is not None:
            if cv2.contourArea(corners.astype(np.float32))<.5*cv2.contourArea(np.asarray(guess,np.float32)):
                return
        if SETTINGS.get('continuous_course') and getattr(self,'gate_number',1)==1 and self.locked is None:
            if np.min(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1))<16.:
                return
        if SETTINGS.get('visual_gravity'):
            measured=self.gravity.observe(stamp,corners,rotation,pose[0],getattr(self,'gate_number',1))
            if measured is not None:
                correction,data=measured
                self.inertial.correct_orientation(data['start'],correction)
                for observer in (self.camera_motion,self.partial_motion):
                    observer.observations=[(t,correction.apply(p) if t>=data['start'] else p) for t,p in observer.observations]
                rotation=correction.as_matrix() @ rotation
                self.gravity_observation=dict(data,image_id=frame.id,capture_time=stamp)
        if SETTINGS.get('partial_gate'):self.rim_template.observe(frame.bgr,corners,pose,frame.id)
        center = position + rotation @ pose[2]
        if self.locked is not None and not SETTINGS.get('continuous_course'):
            limit = max(.5, .05*np.linalg.norm(self.anchor-position))
            if np.linalg.norm(center-self.anchor) > limit:
                return
        self.previous = (position, rotation, corners, pose)
        self.count += 1
        self.centers.append(center)
        self.centers = self.centers[-5:]
        if not SETTINGS.get("camera_correction") or not self.camera_fusion_started:
            self.anchor = np.median(self.centers, axis=0)
        self.last_image_time, self.last_image_id = stamp, frame.id
        self.full_image_time=stamp
        self.association_counts['full_updates']+=1
        self.association_counts['tracked_full_updates']+=int(tracked)
        self.last_corners = corners.copy()
        self.last_relative_gate=rotation @ pose[2]
        self.last_relative_position=np.array(position).copy()
        self.last_gate_corners = pose[1] @ rotation.T + position
        self.used_frames.append(frame.id)
        if self.locked is None and self.count >= SETTINGS["lock_frames"]:
            self.locked = self.track
            self.direction = self.anchor*np.array([1, 1, 0])
            self.direction /= np.linalg.norm(self.direction)
            self.began = self.inertial.time
            if self.phase is not Phase.SHIFTING:self.phase = Phase.TRACKING
        if SETTINGS.get("camera_correction"):
            measured = -rotation @ pose[2]
            side_px = float(np.min(np.linalg.norm(np.roll(corners, -1, axis=0)-corners, axis=1)))
            trusted = self.locked is not None and side_px >= SETTINGS.get('camera_position_min_side_px',60.) and pose[0] <= 1.
            if trusted and not self.camera_fusion_started:
                if not SETTINGS.get('continuous_course'):
                    self.camera_motion.observations.clear()
                self.camera_fusion_started = True
            sampled=trusted or (SETTINGS.get('continuous_course') and self.locked is not None and side_px>=12. and pose[0]<=1.)
            velocity = self.camera_motion.observe(stamp, measured) if sampled else None
            delta_v = np.zeros(3)
            bearing=self.camera_motion.bearing_motion(self.direction) if sampled and SETTINGS.get('continuous_course') else None
            velocity_side=60. if self.phase is Phase.SHIFTING else SETTINGS.get('camera_velocity_min_side_px',60.)
            if bearing is not None and side_px<velocity_side:
                vf=float(self.inertial.velocity_at(bearing['time']) @ self.direction)
                measured_v=transverse_velocity(bearing,self.direction,vf)
                if np.linalg.norm(measured_v)<=8.:
                    delta_v=self.inertial.correct_velocity(bearing['time'],measured_v,.15)
            if velocity is not None and side_px>=velocity_side and np.linalg.norm(velocity["velocity"]) <= 8.:
                gain=.15
                if SETTINGS.get('continuous_course'):
                    gain*=min(1.,.15/max(velocity['residual_m'],.15))
                delta_v = self.inertial.correct_velocity(velocity["time"], velocity["velocity"],gain)
            delta_p = self.inertial.correct_position(stamp, self.anchor+measured, .2) if trusted else np.zeros(3)
            self.camera_observation = dict(image_id=frame.id, capture_time=stamp, used_at=self.inertial.time,
                position_relative_gate=measured.tolist(), velocity=velocity,
                position_correction=delta_p.tolist(), velocity_correction=delta_v.tolist(),
                corners=corners.tolist(), reprojection_rmse_px=float(pose[0]),
                minimum_side_px=side_px, correction_enabled=trusted)
            if SETTINGS.get('continuous_course'):
                self.camera_observation['bearing_motion']=bearing
            self.previous = (self.inertial.camera_pose(stamp)[0], rotation, corners, pose)
            self.last_relative_position=self.inertial.camera_pose(stamp)[0].copy()

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
            visual_setup=SETTINGS.get('visual_initialization',False)
            if visual_setup:
                self.pending.extend(frames)
                for frame in self.pending:
                    stamp=frame.time_ns*1e-9;pose=self.inertial.camera_pose(stamp)
                    if pose is not None:self.observe(frame,stamp,*pose)
                self.pending=[frame for frame in self.pending if frame.time_ns*1e-9>now]
                if self.locked is None:
                    if now-self.setup_started>5.:raise TimeoutError('initial gate unavailable before arming')
                    return None
            target = np.array([0., SETTINGS["shift_right_m"], -1.])
            distance = np.linalg.norm(target-self.inertial.position)
            steady = distance < .04 and np.linalg.norm(self.inertial.velocity) < .08
            if visual_setup:
                target[0]=self.inertial.position[0]
                distance=np.linalg.norm((target-self.inertial.position)[1:])
                start_speed=.5 if SETTINGS.get('moving_start') else .15
                steady=distance<.1 and np.linalg.norm(self.inertial.velocity[1:])<start_speed and abs(self.inertial.velocity[0])<.5
                if SETTINGS.get('moving_start'):steady=steady and now-self.measurement_time()<.2
                if SETTINGS.get('direct_launch'):
                    h=getattr(self.inertial,'history',None)
                    steady=(h is not None and h.filter.airborne and self.inertial.position[2]<-.4
                        and abs(target[1]-self.inertial.position[1])<.2 and np.linalg.norm(self.inertial.velocity)<2.
                        and now-self.measurement_time()<.2)
            self.setup_steady = (self.setup_steady or now) if steady else None
            setup_complete=self.setup_steady is not None and now-self.setup_steady>=(.1 if SETTINGS.get('direct_launch') else .4)
            if now-self.setup_started > 10. and not setup_complete:
                raise TimeoutError("IMU shift and settle exceeded ten seconds")
            if setup_complete:
                self.shift_displacement = self.inertial.position.copy()
                if not SETTINGS.get('continuous_origin'):
                    if visual_setup:
                        self.anchor-=self.shift_displacement
                        self.direction=self.anchor*[1.,1.,0.];self.direction/=np.linalg.norm(self.direction)
                        self.last_relative_position-=self.shift_displacement
                        p,q,c,pose=self.previous
                        self.previous=(p-self.shift_displacement,q,c,pose)
                        self.last_gate_corners-=self.shift_displacement
                        self.centers=[c-self.shift_displacement for c in self.centers]
                        self.camera_motion.observations.clear();self.camera_observation=None
                    if hasattr(self.inertial,'reset_origin'):self.inertial.reset_origin(self.shift_displacement)
                    self.inertial.position = np.zeros(3)
                    self.inertial.times = [now]
                    self.inertial.positions = [self.inertial.position.copy()]
                    self.inertial.velocities = [self.inertial.velocity.copy()]
                    self.inertial.raw_position = np.zeros(3)
                    self.inertial.rotations = [self.inertial.rotation.as_rotation_matrix() @ CAMERA_FROM_BODY.T]
                elif visual_setup:
                    self.direction=(self.anchor-self.inertial.position)*[1.,1.,0.]
                    self.direction/=np.linalg.norm(self.direction)
                self.phase = Phase.TRACKING if visual_setup else Phase.ACQUIRING
                if visual_setup:self.began=now
                self.acquire_started = now
                self.pending = []
            else:
                self.signal = dict(phase=self.phase.name, imu_time=now, position=self.inertial.position.tolist(),
                    velocity=self.inertial.velocity.tolist(), target=target.tolist(),
                    motion_available=False, attitude_available=False)
                if visual_setup:self.signal.update(camera_orientation=self.inertial.rotations[-1].tolist(),camera_observation=self.camera_observation)
                return self.drive(target)
        self.pending.extend(frames)
        for frame in self.pending:
            stamp = frame.time_ns*1e-9
            pose = self.inertial.camera_pose(stamp)
            if pose is not None and (self.phase is not Phase.FINAL_IMU or (SETTINGS.get('continuous_path') and self.gate_remaining()>.15)):
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
        remaining = self.gate_remaining()
        if self.phase is Phase.TRACKING and remaining <= SETTINGS["final_distance_m"] and self.final_approach_ready():
            self.phase, self.final_at = Phase.FINAL_IMU, now
        stop_beyond=SETTINGS.get('finish_beyond_m',SETTINGS['stop_beyond_m']) if getattr(self,'gate_number',1)==SETTINGS.get('gates_to_pass',1) else SETTINGS['stop_beyond_m']
        if remaining < -stop_beyond:
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
        if SETTINGS.get('continuous_course'):
            self.signal['remaining_source']='camera_range_and_velocity'
        if SETTINGS.get("camera_correction"):
            self.signal.update(camera_observation=self.camera_observation,
                unfused_position=self.inertial.raw_position.tolist(), unfused_velocity=self.inertial.raw_velocity.tolist())
        return self.drive(target)
