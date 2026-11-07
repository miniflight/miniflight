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

    def observe_ahead(self, frame, stamp, position, rotation):
        if self.gate_number != 1 or not SETTINGS["lookahead_enabled"] or self.locked is None:
            return
        if (self.anchor-position) @ self.direction > SETTINGS["lookahead_start_m"]:
            return
        if stamp-self.lookahead_scan_at < SETTINGS["lookahead_interval_s"]:
            return
        self.lookahead_scan_at = stamp
        candidates = list(single.vision.aperture_candidates(frame.bgr, close_gaps=True))
        candidates.sort(key=lambda corners: cv2.contourArea(np.asarray(corners, np.float32)), reverse=True)
        for corners in candidates:
            if vision.iou(np.asarray(corners), self.last_corners) > .3:
                continue
            pose = single.vision.best_pose(corners)
            center = position+rotation @ pose[2]
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
            self.signal = dict(phase="HANDOFF", imu_time=self.inertial.time,
                position=self.inertial.position.tolist(), velocity=self.inertial.velocity.tolist(),
                target=self.handoff_target.tolist(), camera_orientation=self.inertial.rotations[-1].tolist(),
                shift_displacement=self.shift_displacement.tolist(), motion_available=False, attitude_available=False)
            handoff_target = self.handoff_target
            if self.lookahead_count >= SETTINGS["lock_frames"]:
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
                self.used_frames.extend(self.lookahead_frames)
                self.began = self.inertial.time
                self.phase = single.Phase.TRACKING
                handoff_target = self.anchor+SETTINGS["beyond_m"]*self.direction
                self.signal["target"] = handoff_target.tolist()
            request = self.drive(handoff_target)
        if self.gate_number == 1 and self.phase is single.Phase.FINAL_IMU and SETTINGS["lookahead_during_final"]:
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
            self.signal.update(gate_number=self.gate_number, estimated_gates_passed=len(self.completed),
                handoff_at=self.handoff_at, imu_history_start=self.inertial.times[0],
                imu_history_samples=len(self.inertial.times), completed_estimates=self.completed)
            if self.lookahead is not None:
                self.signal["lookahead"] = dict(count=self.lookahead_count, image_id=self.lookahead["frame"],
                    image_time=self.lookahead["time"], center=self.lookahead["center"].tolist(),
                    image_corners=self.lookahead["corners"].tolist())
        if self.gate_number == 2 and self.locked is not None and self.inertial.time-self.last_image_time > SETTINGS["lookahead_max_age_s"]:
            raise TimeoutError("second gate image exceeded the lookahead freshness interval")
        return request


class Controller(single.BaseController):
    targets = ("vq1.r1",)

    def __init__(self):
        self.gate = GateController()
        self.motor = single.Motors(position=self.gate)

    def update(self, telemetry, frames, race_state):
        return self.motor.update(telemetry, frames, None)
