"""Streaming camera/IMU window with revisable landmark depths."""
import cv2
import numpy as np

from miniflight.joint import solve_window
from miniflight.odometry import ImuIntegral
from miniflight.vision import landmark_covariance, triangulate_views
from target.aigp.experiments.patch_track import seeds, Tracker
from . import vision


class JointEstimator:
    def __init__(self, call=None, period=.15, recent=5, points=24):
        self.call = call or (lambda name, args, function:function())
        self.period, self.recent, self.maximum_points = period, recent, points
        self.integral = ImuIntegral()
        self.reference = None
        self.ready = False
        self.initial_frames, self.gates = [], []
        self.solutions, self.depth_revisions = 0, 0
        self.last_time, self.last_id = None, None
        self.timings = []

    def imu(self, stamp, dt, specific_force_world):
        self.integral.append(stamp, dt, specific_force_world)

    def between(self, first, last):
        duration, dv, dp = self.integral.between(first, last)
        return duration, self.reference.T @ dv, self.reference.T @ dp

    def _image(self, image, camera_to_world):
        homography = vision.K @ self.reference.T @ camera_to_world @ vision.K_INV
        gray = cv2.warpPerspective(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), homography, (640, 360))
        blue, green, red = cv2.split(image.astype(float))
        mask = cv2.warpPerspective(np.uint8(red-(blue+green)/2 > 20), homography, (640, 360), flags=cv2.INTER_NEAREST)
        valid = cv2.warpPerspective(np.ones((360, 640), np.uint8), homography, (640, 360), flags=cv2.INTER_NEAREST)
        return gray, cv2.dilate(mask, np.ones((19, 19), np.uint8)), cv2.erode(valid, np.ones((19, 19), np.uint8))

    def observe(self, frame, camera_to_world, gate_translation=None):
        stamp = frame.time_ns*1e-9
        if self.last_id is not None and (frame.id <= self.last_id or stamp <= self.last_time):
            return
        if self.reference is None:
            if gate_translation is None:
                return
            self.reference = np.array(camera_to_world)
            self.reference_time, self.reference_id = stamp, frame.id
            self.reference_tau = self.integral.at(stamp)[0]
            gray, mask, valid = self._image(frame.bgr, camera_to_world)
            candidates = [p for p in seeds(gray) if not mask[round(p[1]), round(p[0])] and valid[round(p[1]), round(p[0])]]
            self.tracker = Tracker(gray, candidates)
        else:
            gray, mask, valid = self._image(frame.bgr, camera_to_world)
            self.tracker.advance(gray)
        for point in np.flatnonzero(self.tracker.alive):
            x, y = np.round(self.tracker.points[point]).astype(int)
            if mask[y, x] or not valid[y, x]:
                self.tracker.alive[point] = False
        self.last_id, self.last_time = frame.id, stamp
        self.last_camera_rotation = np.array(camera_to_world)
        if not self.ready:
            tau = self.integral.at(stamp)[0]-self.reference_tau
            self.initial_frames.append(dict(id=frame.id, time=stamp, tau=tau, pixels=self.tracker.points.copy()))
            if gate_translation is not None:
                self.gates.append((stamp, tau, self.reference.T @ camera_to_world @ gate_translation))
            if tau >= 2. and len(self.initial_frames) >= 40:
                self._initialize()
            return
        if stamp-self.window[-1]['time'] < self.period:
            return
        self._step(frame.id, stamp)

    def _initialize(self):
        if len(self.gates) < 12 or self.last_time-self.gates[-1][0] > .1:
            raise RuntimeError('joint initialization lost its gate scale observations')
        times = np.array([g[1] for g in self.gates])
        values = np.array([g[2] for g in self.gates])
        design = np.column_stack((np.ones(len(times)), times))
        weights = np.ones(len(times))
        for _ in range(4):
            root = np.sqrt(weights)
            fit = np.linalg.lstsq(design*root[:, None], values*root[:, None], rcond=None)[0]
            errors = np.linalg.norm(design @ fit-values, axis=1)
            weights = np.minimum(1., .08/np.maximum(errors, 1e-9))
        self.initial_velocity = -fit[1]
        key = np.unique(np.round(np.linspace(0, len(self.initial_frames)-1, 12)).astype(int))
        positions = np.array([self.initial_frames[i]['tau']*self.initial_velocity for i in key])
        rotations = np.repeat(np.eye(3)[None], len(key), axis=0)
        candidates = []
        for index in np.flatnonzero(self.tracker.alive):
            pixels = np.array([self.initial_frames[i]['pixels'][index] for i in key])
            parallax = np.linalg.norm(pixels[-1]-pixels[0])
            if parallax < 1.5:
                continue
            point = triangulate_views(pixels, rotations, positions, vision.K)
            if point is None or point[2] <= .5:
                continue
            camera = point-positions
            h = camera @ vision.K.T
            rms = np.sqrt(np.mean((h[:, :2]/h[:, 2:]-pixels)**2))
            if rms > 1.:
                continue
            covariance = landmark_covariance(point, rotations, positions, vision.K, max(.3, rms))
            if covariance is None:
                continue
            sigma = np.clip(np.sqrt(covariance[2, 2])/point[2], .15, .5)
            cell = tuple((self.tracker.seeds[index, 0]/[160, 120]).astype(int))
            candidates.append((index, point, float(sigma), float(parallax), cell))
        groups = {}
        for item in sorted(candidates, key=lambda value:-value[3]):
            groups.setdefault(item[4], []).append(item)
        selected = []
        while groups and len(selected) < self.maximum_points:
            for cell in list(groups):
                selected.append(groups[cell].pop(0))
                if not groups[cell]:
                    del groups[cell]
                if len(selected) == self.maximum_points:
                    break
        if len(selected) < 8:
            raise RuntimeError('joint initialization has fewer than eight observable background points')
        ids = [item[0] for item in selected]
        tracker = Tracker(self.tracker.reference, self.tracker.seeds[ids, 0])
        tracker.points = self.tracker.points[ids].copy()
        self.tracker = tracker
        world = np.array([item[1] for item in selected])
        self.rays = world/world[:, 2:]
        self.depth_prior = np.log(world[:, 2])
        self.initial_depth_prior = self.depth_prior.copy()
        self.depth_sigma = np.array([item[2] for item in selected])
        self.depths = self.depth_prior.copy()
        self.anchor = np.r_[self.initial_frames[-1]['tau']*self.initial_velocity, self.initial_velocity]
        duration, force, _ = self.between(self.reference_time, self.last_time)
        self.gravity = -force/duration
        self.window = [dict(id=self.last_id, time=self.last_time, ids=[], pixels=[])]
        self.states = {self.last_id:np.r_[self.anchor, np.zeros(3)]}
        self.solution_time = self.last_time
        self.ready = True
        self.initial_frames = []

    def predict(self, stamp):
        if not self.ready:
            return None
        state = self.states[self.window[-1]['id']]
        dt, dv, dp = self.between(self.solution_time, stamp)
        correction = cv2.Rodrigues(state[6:9])[0].T
        return np.r_[state[:3]+state[3:6]*dt+correction @ dp+.5*self.gravity*dt**2,
                     state[3:6]+correction @ dv+self.gravity*dt, state[6:9]]

    def _step(self, image_id, stamp):
        ids = np.flatnonzero(self.tracker.alive).tolist()
        if len(ids) < 6:
            raise RuntimeError('joint estimator lost its background tracks')
        predicted = self.predict(stamp)
        self.window.append(dict(id=image_id, time=stamp, ids=ids, pixels=self.tracker.points[ids].tolist()))
        self.states[image_id] = predicted
        self.window = self.window[:1]+self.window[1:][-self.recent:]
        self.states = {frame['id']:self.states[frame['id']] for frame in self.window}
        edges = []
        for i in range(len(self.window)-1):
            dt, dv, dp = self.between(self.window[i]['time'], self.window[i+1]['time'])
            edges.append(dict(first=i, last=i+1, dt=float(dt), dv=dv.tolist(), dp=dp.tolist()))
        depth_prior = self.depth_prior.copy()
        observed = {point for frame in self.window for point in frame['ids']}
        inactive = [point for point in range(len(depth_prior)) if point not in observed]
        depth_prior[inactive] = self.depths[inactive]
        request = dict(intrinsics=vision.K.tolist(), rays=self.rays.tolist(), depth_prior=depth_prior.tolist(),
            depth_sigma=self.depth_sigma.tolist(), gravity=self.gravity.tolist(), anchor=self.anchor.tolist(),
            states=[self.states[f['id']].tolist() for f in self.window], depths=self.depths.tolist(),
            frames=[dict(ids=f['ids'], pixels=f['pixels']) for f in self.window], edges=edges)
        reply = self.call('joint_solve', dict(image_id=int(image_id), capture_ns=round(stamp*1e9), sequence=self.solutions+1),
                          lambda:dict(request=request, fit=solve_window(request)))
        saved, fit = reply['request'], reply['fit']
        for key in ('intrinsics', 'rays', 'depth_prior', 'depth_sigma', 'gravity', 'anchor', 'states', 'depths'):
            np.testing.assert_allclose(saved[key], request[key], rtol=1e-8, atol=1e-6)
        for a, b in zip(saved['frames'], request['frames'], strict=True):
            assert a['ids'] == b['ids']
            np.testing.assert_allclose(a['pixels'], b['pixels'], rtol=0, atol=1e-4)
        for a, b in zip(saved['edges'], request['edges'], strict=True):
            assert (a['first'], a['last']) == (b['first'], b['last'])
            np.testing.assert_allclose([a['dt'], *a['dv'], *a['dp']], [b['dt'], *b['dv'], *b['dp']], atol=1e-6)
        if not fit['finite'] or fit['reprojection_rms_px'] > 1.5 or fit['solve_ms'] > 150:
            raise RuntimeError('joint solution failed its finite, pixel-error or runtime check')
        fitted = np.array(fit['states'])
        if np.linalg.norm(fitted[-1, :3]-predicted[:3]) > .5 or np.linalg.norm(fitted[-1, 3:6]) > 4.:
            raise RuntimeError('joint solution exceeded its correction bound')
        self.depth_revisions += int(np.max(abs(np.array(fit['depths'])-saved['depths'])) > 1e-6)
        if not self.solutions:
            self.initial_depth_prior = np.array(saved['depth_prior'])
        for name in ('rays', 'depth_prior', 'depth_sigma', 'gravity', 'anchor'):
            setattr(self, name, np.array(saved[name]))
        self.depths = np.array(fit['depths'])
        for frame, state in zip(self.window, fitted, strict=True):
            self.states[frame['id']] = state
        self.solution_time = stamp
        self.solutions += 1
        self.timings.append(fit['solve_ms'])
        self.last_fit = fit
        self.depth_change = float(np.median(abs(np.exp(self.depths[ids]-self.initial_depth_prior[ids])-1)))
