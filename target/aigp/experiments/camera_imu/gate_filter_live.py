"""Experimental joint feedback and one-gate braking test on public sensors."""
from bisect import bisect_left
import numpy as np
from scipy.spatial.transform import Rotation
from common.math import Quaternion
from miniflight.gate_filter import GateFilter,GateHistory
from miniflight.profiling import timed
from . import course,sensors,vision


class Inertial(sensors.Inertial):
    def __init__(self,config):
        super().__init__();self.config=config;self.history=None;self.serial=0

    @timed('prediction')
    def update(self,samples):
        for sample in samples:
            if self.history is None:
                super().update([sample])
                if self.rotation is None:continue
                rates=np.array([s.gyro for s in self.seed]);f=GateFilter(self.rotation.as_rotation_matrix(),np.zeros(3),
                    self.config.thrust_curve,self.config.body_resistance_per_m,vision.CAMERA_FROM_BODY,vision.K)
                f.gyro_noise=max(.001,float(max(rates.std(axis=0))*np.sqrt(np.median(np.diff([s.time for s in self.seed])))))
                self.history=GateHistory(f,self.time,sample.gyro,seconds=2.)
            elif sample.time>self.time:
                self.history.imu(sample.time,sample.gyro,sample.acceleration);self.times.append(sample.time)
                self.positions.append(np.zeros(3));self.velocities.append(np.zeros(3));self.rotations.append(np.eye(3))
            self.gyro=sample.gyro;self.sync()

    def sync(self):
        h=self.history;f=h.filter;self.time=h.time;self.position=f.p.copy();self.velocity=f.v.copy();self.prediction_bias=f.ba.copy()
        x,y,z,w=Rotation.from_matrix(f.R).as_quat();self.rotation=Quaternion(w,x,y,z)
        snapshots={t:s for t,s in h.snapshots}
        for i in range(bisect_left(self.times,h.snapshots[0][0]),len(self.times)):
            state=snapshots.get(self.times[i])
            if state is not None:
                self.positions[i]=state.p.copy();self.velocities[i]=state.v.copy();self.rotations[i]=state.R @ vision.CAMERA_FROM_BODY.T

    def reset_origin(self,shift):
        for state in [self.history.filter]+[s for _,s in self.history.snapshots]:
            state.p-=shift
            state.gates={k:(i,g-shift,yaw) for k,(i,g,yaw) in state.gates.items()}
            state.proposals={k:[(identity,t,center-shift) for identity,t,center in rows] for k,rows in state.proposals.items()}
        self.positions=[p-shift for p in self.positions];self.sync()

    @timed('joint_pixel_update')
    def pixels(self,key,stamp,image_id,pixels,template=None):
        seed=None;rim=template is not None
        if not rim:
            pose=vision.best_pose(pixels)
            if pose is None:return
            seed=(pose[2],(pose[1][1]-pose[1][0])/1.5)
        points=None if rim else .75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])
        identity=self.serial
        self.history.observe(dict(id=identity,image_id=image_id,time=stamp,key=key,pixels=np.asarray(pixels),points=points,seed=seed,
            rim=rim,rim_prior=None if template is None else (template['width_m'],template['center_to_top_m'],template['fractional_spread'])))
        self.serial+=1;self.sync()
        return self.history.results.get(identity)

    def correct_velocity(self,*args):return np.zeros(3)
    def correct_position(self,*args):return np.zeros(3)
    def correct_orientation(self,*args):raise AssertionError('Complementary attitude correction entered joint mode')


class GateController(course.GateController):
    def __init__(self,calls):
        super().__init__(calls);self.inertial=Inertial(self.config);self.inertial.timing=self.timing
        self.brake_state='waiting';self.brake_steady=None;self.used_joint=set()

    @property
    def anchor(self):
        h=getattr(getattr(self,'inertial',None),'history',None);key=getattr(self,'gate_number',1)
        if h is not None and key in h.filter.gates:return h.filter.gates[key][1].copy()
        return getattr(self,'_anchor',None)

    @anchor.setter
    def anchor(self,value):self._anchor=value

    def measurement_time(self):
        h=getattr(self.inertial,'history',None)
        if h is not None and self.gate_number in h.filter.gates:return h.filter.last_visual.get(self.gate_number,-np.inf)
        return super().measurement_time()

    def gate_relative(self):
        h=self.inertial.history
        if h is not None and self.gate_number in h.filter.gates:return h.filter.relative(self.gate_number)[0]
        return super().gate_relative()

    def update(self,telemetry,frames,race_state):
        request=super().update(telemetry,frames,None)
        # A confirmed lookahead can become current before another image arrives.
        h=getattr(self.inertial,'history',None)
        if h is not None and self.locked is not None and self.gate_number not in h.filter.gates and self.last_image_id is not None:
            self.inertial.pixels(self.gate_number,self.last_image_time,self.last_image_id,self.last_corners)
        if self.locked is not None and self.phase is sensors.Phase.TRACKING and self.inertial.time-self.measurement_time()>2.:
            raise TimeoutError('joint gate measurement exceeded two seconds')
        return request

    def observe(self,frame,stamp,position,rotation):
        super().observe(frame,stamp,position,rotation)
        if self.locked is None:return
        full=self.last_image_id==frame.id
        partial=self.partial_observation
        if full and (frame.id,'full',self.gate_number) not in self.used_joint:
            self.used_joint.add((frame.id,'full',self.gate_number));self.inertial.pixels(self.gate_number,stamp,frame.id,self.last_corners)
            pose=vision.best_pose(self.last_corners)
            self.camera_observation=dict(image_id=frame.id,capture_time=stamp,used_at=self.inertial.time,
                position_relative_gate=(-rotation @ pose[2]).tolist(),corners=self.last_corners.tolist(),
                reprojection_rmse_px=float(pose[0]),minimum_side_px=float(np.min(np.linalg.norm(self.last_corners-np.roll(self.last_corners,1,axis=0),axis=1))),
                velocity=dict(time=self.inertial.time,velocity=self.inertial.velocity.tolist(),source='joint posterior'),
                position_correction=[0.]*3,velocity_correction=[0.]*3,correction_enabled=False)
        if partial is not None and partial['image_id']==frame.id and partial['gate_number']==self.gate_number and (frame.id,'top',self.gate_number) not in self.used_joint:
            self.used_joint.add((frame.id,'top',self.gate_number));self.inertial.pixels(self.gate_number,stamp,frame.id,partial['top_corners'],partial['template'])
            partial['velocity']=dict(time=self.inertial.time,velocity=self.inertial.velocity.tolist(),source='joint posterior')
        f=self.inertial.history.filter
        if self.gate_number in f.gates:
            self.anchor=f.gates[self.gate_number][1].copy();self.last_relative_gate=self.gate_relative();self.last_relative_position=f.p.copy()
            self.full_image_time=f.last_full_visual.get(self.gate_number)

    def drive(self,target):
        if sensors.SETTINGS.get('brake_test') and self.locked is not None and self.phase is sensors.Phase.TRACKING:
            if self.brake_state=='waiting' and self.gate_remaining()<=10.:self.brake_state='braking'
            if self.brake_state=='braking':
                r=self.gate_relative();d=self.direction;v=self.inertial.velocity
                settled=abs(v @ d)<.3 and np.linalg.norm(v)<.4 and np.linalg.norm(r-(r @ d)*d)<.3 and self.inertial.time-self.measurement_time()<.3
                self.brake_steady=(self.inertial.time if self.brake_steady is None else self.brake_steady) if settled else None
                if self.brake_steady is not None and self.inertial.time-self.brake_steady>=.4:self.brake_state='resumed'
                else:
                    target=self.inertial.position+r-(r @ d)*d;self.signal.update(target=target.tolist(),brake_test_active=True)
                    return sensors.GateController.drive(self,target)
        self.signal['brake_test_active']=False
        return super().drive(target)


class Controller(course.Controller):
    def __init__(self,calls):
        self.gate=GateController(calls);self.motor=sensors.Motors(position=self.gate)
        self.timing=self.gate.timing;self.motor.timing=self.timing

    def update(self,telemetry,frames,race_state):
        command=super().update(telemetry,frames,None)
        h=self.gate.inertial.history
        if command is not None and h is not None:
            f=h.filter
            if self.gate.gate_number in f.gates:
                relative,cov,camera,camcov=f.relative(self.gate.gate_number)
                self.gate.signal['joint_posterior']=dict(position_covariance=f.P[:3,:3].tolist(),velocity_covariance=f.P[3:6,3:6].tolist(),
                    attitude_covariance=f.P[6:9,6:9].tolist(),relative_gate=relative.tolist(),relative_covariance=cov.tolist(),camera_gate=camera.tolist(),camera_covariance=camcov.tolist(),
                    gyro_bias=f.bg.tolist(),force_bias=f.ba.tolist(),aperture_size_m=f.size,retargets=f.retargets)
                self.gate.signal['joint_posterior'].update(gate_yaw=f.gates[self.gate.gate_number][2],last_accepted_image_time=f.last_visual.get(self.gate.gate_number),
                    position_gate_covariance=f.P[:3,f.gates[self.gate.gate_number][0]:f.gates[self.gate.gate_number][0]+3].tolist())
            self.gate.signal['brake_test_state']=self.gate.brake_state
            h.command(h.time,command.outputs);self.gate.inertial.sync()
        return command
