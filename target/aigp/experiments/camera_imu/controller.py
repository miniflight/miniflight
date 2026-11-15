"""Acquire a second gate after the first pass, retaining the same IMU state."""


import numpy as np
import cv2


from . import sensors as single


ALLOWED = single.ALLOWED
vision = single.vision
CameraClock, SensorClient = single.CameraClock, single.SensorClient
SETTINGS = single.SETTINGS
SETTINGS.update(gates_to_pass=2, beyond_m=4., handoff_distance_m=5., lookahead_enabled=True,
                lookahead_start_m=12., lookahead_interval_s=.1, lookahead_max_age_s=2.,
                lookahead_during_final=True, height_alignment=True,
                height_alignment_distance_m=6., height_alignment_deadband_m=.4)


def acquisition_waypoint(position,observed):
    horizontal=(observed-position)*[1.,1.,0.]
    distance=np.linalg.norm(horizontal)
    target=position+horizontal*min(1.,SETTINGS['height_alignment_distance_m']/max(distance,1e-9))
    target[2]=observed[2]
    return target


class GateController(single.GateController):
    def __init__(self):
        super().__init__()
        self.gate_number = 1
        self.completed = []
        self.handoff_at = None
        self.handoff_target = None
        self.lookahead = None
        self.lookahead_centers = []
        self.lookahead_frames = []
        self.lookahead_count = 0
        self.lookahead_scan_at = -float("inf")
        self.acquisition_prior=None
        self.nearer_centers=[]
        self.nearer_frames=[]
        self.retargets=0

    def nearer_gate(self,frame,stamp,position,rotation,corners,pose,center):
        if self.nearer_centers and np.linalg.norm(center-np.median(self.nearer_centers,axis=0))>max(1.5,.06*np.linalg.norm(center-position)):
            self.nearer_centers,self.nearer_frames=[],[]
        self.nearer_centers.append(center);self.nearer_frames.append(frame.id)
        if len(self.nearer_centers)<SETTINGS['lock_frames']:
            return
        old=self.previous
        if old is not None and vision.iou(corners,self.last_corners)<.6:
            self.lookahead=dict(frame=self.last_image_id,time=self.last_image_time,position=old[0].copy(),
                rotation=old[1].copy(),corners=old[2].copy(),pose=old[3],center=self.anchor.copy())
            self.lookahead_count=self.count
            self.lookahead_centers=list(self.centers)
            self.lookahead_frames=self.used_frames[-min(self.count,5):]
        self.anchor=np.median(self.nearer_centers[-5:],axis=0)
        self.direction=(self.anchor-self.inertial.position)*[1.,1.,0.]
        self.direction/=np.linalg.norm(self.direction)
        self.previous=(position,rotation,corners,pose)
        self.last_image_time,self.last_image_id=stamp,frame.id
        self.last_corners=corners.copy();self.last_gate_corners=pose[1] @ rotation.T+position
        self.last_relative_gate=rotation @ pose[2]
        self.last_relative_position=np.array(position).copy()
        self.count=len(self.nearer_centers);self.centers=list(self.nearer_centers[-5:])
        self.track+=1;self.locked=self.track;self.began=self.inertial.time;self.final_at=None
        self.camera_motion.observations.clear();self.camera_fusion_started=False;self.camera_observation=None
        self.nearer_centers,self.nearer_frames=[],[];self.retargets+=1

    def observe_ahead(self, frame, stamp, position, rotation):
        if (self.gate_number >= SETTINGS["gates_to_pass"] and not SETTINGS.get('continuous_course')) or not SETTINGS["lookahead_enabled"] or self.locked is None:
            return
        if not SETTINGS.get('continuous_course') and (self.anchor-position) @ self.direction > SETTINGS["lookahead_start_m"]:
            return
        if stamp-self.lookahead_scan_at < SETTINGS["lookahead_interval_s"]:
            return
        self.lookahead_scan_at = stamp
        candidates = list(single.vision.aperture_candidates(frame.bgr, close_gaps=True,strict=SETTINGS.get('continuous_course',False)))
        candidates.sort(key=lambda corners: cv2.contourArea(np.asarray(corners, np.float32)), reverse=True)
        for corners in candidates:
            pose = single.vision.best_pose(corners)
            center = position+rotation @ pose[2]
            if (SETTINGS.get('continuous_course') and (center-position) @ self.direction>3.
                    and (center-self.anchor) @ self.direction < -5.):
                self.nearer_gate(frame,stamp,position,rotation,corners,pose,center)
                return
            if vision.iou(np.asarray(corners), self.last_corners) > .3 or self.gate_number>=SETTINGS['gates_to_pass']:
                continue
            if (center-self.anchor) @ self.direction < 5.:
                continue
            if self.lookahead_centers:
                median = np.median(self.lookahead_centers, axis=0)
                limit = max(1.5, .06*np.linalg.norm(center-position))
                if np.linalg.norm(center-median) > limit:
                    farther = (center-median) @ self.direction >= 0
                    if self.lookahead_count >= SETTINGS["lock_frames"] and farther:
                        continue
                    self.lookahead_centers, self.lookahead_frames = [], []
                    self.lookahead_count = 0
            self.lookahead_count += 1
            self.lookahead_centers.append(center)
            self.lookahead_centers = self.lookahead_centers[-5:]
            self.lookahead_frames.append(frame.id)
            self.lookahead = dict(frame=frame.id, time=stamp, position=position.copy(), rotation=rotation.copy(),
                corners=corners.copy(), pose=pose, center=np.median(self.lookahead_centers, axis=0))
            break

    def drive(self, target):
        if self.gate_number > 1 and self.phase is single.Phase.ACQUIRING:
            target = self.handoff_target
        if (self.gate_number > 1 and self.phase is single.Phase.TRACKING and SETTINGS["height_alignment"]
                and abs(self.anchor[2]-self.inertial.position[2]) > SETTINGS["height_alignment_deadband_m"]):
            p = self.inertial.position
            horizontal = (self.anchor-p)*[1, 1, 0]
            distance = np.linalg.norm(horizontal)
            target = p+horizontal*min(1., SETTINGS["height_alignment_distance_m"]/max(distance, 1e-9))
            target[2] = self.anchor[2]
            self.signal["target"] = target.tolist()
            self.signal["height_alignment"] = True
        return super().drive(target)

    def observe(self, frame, stamp, position, rotation):
        if self.handoff_at is not None and stamp <= self.handoff_at:
            return
        was_locked = self.locked is not None
        super().observe(frame, stamp, position, rotation)
        if self.phase is single.Phase.SHIFTING:return
        if SETTINGS.get('visual_gravity'):
            corrected=self.inertial.camera_pose(stamp)
            if corrected is not None:position,rotation=corrected
        if not was_locked and self.locked is not None:
            self.direction = (self.anchor-position)*[1, 1, 0]
            self.direction /= np.linalg.norm(self.direction)
        self.observe_ahead(frame, stamp, position, rotation)

    def update(self, telemetry, frames, race_state):
        ahead_frames = list(self.pending)+list(frames)
        try:
            request = super().update(telemetry, frames, None)
        except StopIteration:
            self.completed.append(dict(gate=self.gate_number, imu_time=self.inertial.time,
                estimated_center=self.anchor.tolist(), position=self.inertial.position.tolist(),
                velocity=self.inertial.velocity.tolist(), image_id=self.last_image_id,
                image_time=self.last_image_time))
            if self.gate_number == SETTINGS["gates_to_pass"]:
                self.signal.update(estimated_gates_passed=len(self.completed), completed_estimates=self.completed)
                raise
            self.handoff_target = self.anchor+SETTINGS["handoff_distance_m"]*self.direction
            self.handoff_at = self.inertial.time
            self.acquisition_prior=self.lookahead
            self.gate_number += 1
            self.phase = single.Phase.ACQUIRING
            self.acquire_started = self.handoff_at
            self.previous = self.locked = None
            self.count = 0
            self.centers = []
            self.anchor = self.direction = None
            self.pending = []
            self.last_image_time = self.last_image_id = None
            self.last_corners = self.last_gate_corners = None
            self.final_at = self.began = None
            if SETTINGS.get('continuous_course'):
                self.camera_motion.observations.clear()
                self.camera_fusion_started=False
                self.camera_observation=None
                self.last_relative_gate=None
                self.last_relative_position=None
                self.nearer_centers,self.nearer_frames=[],[]
            self.signal = dict(phase="HANDOFF", imu_time=self.inertial.time,
                position=self.inertial.position.tolist(), velocity=self.inertial.velocity.tolist(),
                target=self.handoff_target.tolist(), camera_orientation=self.inertial.rotations[-1].tolist(),
                shift_displacement=self.shift_displacement.tolist(), motion_available=False, attitude_available=False)
            handoff_target = self.handoff_target
            fresh = self.lookahead is not None and self.inertial.time-self.lookahead['time'] <= SETTINGS['lookahead_max_age_s']
            if self.lookahead_count >= SETTINGS["lock_frames"] and (fresh or not SETTINGS.get('continuous_course')):
                observed = self.lookahead
                self.anchor = observed["center"].copy()
                self.direction = (self.anchor-self.inertial.position)*[1, 1, 0]
                self.direction /= np.linalg.norm(self.direction)
                self.previous = (observed["position"], observed["rotation"], observed["corners"], observed["pose"])
                self.track += 1
                self.locked = self.track
                self.count = self.lookahead_count
                self.centers = list(self.lookahead_centers)
                self.last_image_time, self.last_image_id = observed["time"], observed["frame"]
                self.last_corners = observed["corners"].copy()
                self.last_gate_corners = observed["pose"][1] @ observed["rotation"].T+observed["position"]
                self.last_relative_gate=observed['rotation'] @ observed['pose'][2]
                self.last_relative_position=observed['position'].copy()
                self.used_frames.extend(self.lookahead_frames)
                self.began = self.inertial.time
                self.phase = single.Phase.TRACKING
                handoff_target = self.anchor+SETTINGS["beyond_m"]*self.direction
                self.signal["target"] = handoff_target.tolist()
            elif SETTINGS.get('continuous_course') and self.lookahead is not None:
                observed=self.lookahead
                self.previous=(observed['position'],observed['rotation'],observed['corners'],observed['pose'])
                self.handoff_target=acquisition_waypoint(self.inertial.position,observed['center'])
                handoff_target=self.handoff_target
            request = self.drive(handoff_target)
            if SETTINGS.get('continuous_course'):
                self.lookahead=None
                self.lookahead_centers,self.lookahead_frames=[],[]
                self.lookahead_count=0
                self.lookahead_scan_at=-float('inf')
        if self.gate_number < SETTINGS['gates_to_pass'] and self.phase is single.Phase.FINAL_IMU and SETTINGS["lookahead_during_final"]:
            for frame in ahead_frames:
                stamp = frame.time_ns*1e-9
                pose = self.inertial.camera_pose(stamp)
                if pose is not None:
                    self.observe_ahead(frame, stamp, *pose)
        if self.gate_number > 1 and self.phase is single.Phase.ACQUIRING:
            self.signal["phase"] = "HANDOFF"
            self.signal["target"] = self.handoff_target.tolist()
            self.signal["camera_orientation"] = self.inertial.rotations[-1].tolist()
        if self.signal:
            if SETTINGS.get('continuous_course'):
                self.signal['closer_gate_corrections']=self.retargets
            self.signal.update(gate_number=self.gate_number, estimated_gates_passed=len(self.completed),
                handoff_at=self.handoff_at, imu_history_start=self.inertial.times[0],
                imu_history_samples=len(self.inertial.times), completed_estimates=self.completed)
            if self.lookahead is not None:
                self.signal["lookahead"] = dict(count=self.lookahead_count, image_id=self.lookahead["frame"],
                    image_time=self.lookahead["time"], center=self.lookahead["center"].tolist(),
                    image_corners=self.lookahead["corners"].tolist())
        if self.gate_number >= 2 and self.phase is single.Phase.TRACKING and self.locked is not None and self.inertial.time-self.last_image_time > SETTINGS["lookahead_max_age_s"]:
            if not (SETTINGS.get('continuous_course') and getattr(self,'alignment_active',False)
                    and self.inertial.time-self.last_image_time<=SETTINGS['maximum_blind_s']):
                raise TimeoutError("gate image exceeded the lookahead freshness interval")
        return request


class Controller(single.BaseController):
    targets = ("vq1.r1",)

    def __init__(self):
        self.gate = GateController()
        self.motor = single.Motors(position=self.gate)

    def update(self, telemetry, frames, race_state):
        return self.motor.update(telemetry, frames, None)
