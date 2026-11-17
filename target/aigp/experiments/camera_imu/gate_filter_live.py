"""Experimental joint feedback and one-gate braking test on public sensors."""
from bisect import bisect_left
import numpy as np
from scipy.spatial.transform import Rotation
from common.math import Quaternion
from miniflight.control import BodyRates
from miniflight.gate_filter import GateFilter,GateHistory
from miniflight.profiling import timed
from . import course,sensors,vision
from .gate_filter_events import state as estimator_state
from .partial import outer_top,rim_from_opening,same_red_component


def full_rim_observation(image,corners,pose):
    top=outer_top(image,corners)
    if top is None:top=outer_top(image)
    dimensions=None if top is None else rim_from_opening(top[0],corners)
    if (dimensions is None or not (1.5<dimensions[0]<6. and .75<dimensions[1]<4.5)
            or not same_red_component(image,corners,top[0])):return None
    return top[0],dict(width_m=float(dimensions[0]),center_to_top_m=float(dimensions[1]),fractional_spread=1.,samples=1)


def outage_budget(relative,covariance,direction,aligning,maximum=4.5):
    """A mapped far-field height maneuver can outlast the normal visual interval."""
    right=np.array([-direction[1],direction[0],0.]);axes=np.array([right,[0.,0.,1.]])
    distance=float(np.asarray(relative) @ direction)
    spread=3.*np.sqrt(max(0.,float(np.linalg.eigvalsh(axes @ covariance @ axes.T)[-1])))
    angle=float(np.arctan2(spread,max(.1,distance)))
    allowed=bool(aligning and distance>8. and angle<.15)
    return dict(budget_s=maximum if allowed else 2.,mapped_height_alignment=allowed,
        range_m=distance,transverse_3sigma_m=float(spread),bearing_3sigma_rad=angle,bearing_limit_rad=.15)


class Inertial(sensors.Inertial):
    def __init__(self,config,record=None):
        super().__init__();self.config=config;self.history=None;self.serial=0;self.record=record;self.input_count=0

    def event(self,kind,**values):
        self.input_count+=1
        if self.record is not None:self.record(event='estimator_input',sequence=self.input_count,kind=kind,**values)

    def command(self,stamp,motors):
        self.event('command',time=float(stamp),motors=list(motors));self.history.command(stamp,motors)

    @timed('prediction')
    def update(self,samples):
        for sample in samples:
            if self.history is None:
                super().update([sample])
                if self.rotation is None:continue
                rates=np.array([s.gyro for s in self.seed]);f=GateFilter(self.rotation.as_rotation_matrix(),np.zeros(3),
                    self.config.thrust_curve,self.config.body_resistance_per_m,vision.CAMERA_FROM_BODY,vision.K,
                    supported=sensors.SETTINGS.get('ground_support',False))
                f.gyro_noise=max(.001,float(max(rates.std(axis=0))*np.sqrt(np.median(np.diff([s.time for s in self.seed])))))
                f.localization=sensors.SETTINGS.get('map_localization',False)
                f.attitude_corrections=f.localization or not sensors.SETTINGS.get('gyro_attitude',False)
                f.broad_rim=sensors.SETTINGS.get('joint_rim',False)
                f.partial_gate_births=not sensors.SETTINGS.get('joint_rim',False)
                f.image_time_sigma=.01 if sensors.SETTINGS.get('image_time_uncertainty',False) else 0.
                f.upright_attitude=sensors.SETTINGS.get('upright_attitude',False)
                prior=sensors.SETTINGS.get('map_prior')
                if prior is not None:f.restore_map(prior)
                self.history=GateHistory(f,self.time,sample.gyro,seconds=2.)
                self.event('init',seed=[[float(s.time),*s.acceleration,*s.gyro] for s in self.seed],config=dict(
                    thrust_curve=list(self.config.thrust_curve),drag=list(self.config.body_resistance_per_m),camera_from_body=vision.CAMERA_FROM_BODY.tolist(),
                    intrinsics=vision.K.tolist(),supported=not f.airborne,localization=f.localization,attitude_corrections=f.attitude_corrections,broad_rim=f.broad_rim,partial_gate_births=f.partial_gate_births,
                    image_time_sigma_s=f.image_time_sigma,upright_attitude=f.upright_attitude,history_s=2.,**({} if prior is None else dict(map_prior=prior))))
            elif sample.time>self.time:
                self.event('imu',time=float(sample.time),gyro=list(sample.gyro),acceleration=list(sample.acceleration))
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
        self.event('pixels',time=float(stamp),gate=int(key),image_id=int(image_id),pixels=np.asarray(pixels).tolist(),template=template)
        self.history.observe(dict(id=identity,image_id=image_id,time=stamp,key=key,pixels=np.asarray(pixels),points=points,seed=seed,
            rim=rim,rim_prior=None if template is None else (template['width_m'],template['center_to_top_m'],template['fractional_spread'])))
        self.serial+=1;self.sync()
        return self.history.results.get(identity)

    def correct_velocity(self,*args):return np.zeros(3)
    def correct_position(self,*args):return np.zeros(3)
    def correct_orientation(self,*args):raise AssertionError('Complementary attitude correction entered joint mode')


class GateController(course.GateController):
    def __init__(self,calls):
        super().__init__(calls);self.inertial=Inertial(self.config,getattr(calls,'record',None));self.inertial.timing=self.timing
        self.brake_state='waiting';self.brake_steady=None;self.used_joint=set();self.launch_cleared=False
        self.reacquire_since=None

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

    def lookahead_admitted(self,observed):
        return self.inertial.history.filter.last_visual.get(self.gate_number)==observed['time']

    def fuse_lookahead(self,frame,stamp):
        observed=self.lookahead;key=self.gate_number+1
        if (observed is None or observed['frame']!=frame.id or self.lookahead_count<3
                or key>sensors.SETTINGS['gates_to_pass'] or (frame.id,'full',key) in self.used_joint):return
        self.used_joint.add((frame.id,'full',key))
        self.inertial.pixels(key,stamp,frame.id,observed['corners'])

    def observe_ahead(self,frame,stamp,position,rotation):
        super().observe_ahead(frame,stamp,position,rotation)
        if self.phase is sensors.Phase.FINAL_IMU:self.fuse_lookahead(frame,stamp)

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
        if self.locked is not None and self.phase is sensors.Phase.TRACKING:
            budget=dict(budget_s=2.)
            if h is not None and self.gate_number in h.filter.gates:
                relative,covariance,_,_=h.filter.relative(self.gate_number)
                budget=outage_budget(relative,covariance,self.direction,self.alignment_active,sensors.SETTINGS['maximum_blind_s'])
            age=float(self.inertial.time-self.measurement_time())
            if getattr(self,'recovery_stop',None) is not None:budget['budget_s']=getattr(self,'recovery_budget_s',sensors.SETTINGS['maximum_blind_s'])
            self.signal['visual_outage']=dict(budget,image_age_s=age)
            if age>budget['budget_s']:
                raise TimeoutError('joint gate measurement exceeded two seconds' if budget['budget_s']==2. else 'joint gate measurement exceeded its mapped visibility budget')
        return request

    def observe_partial(self,frame,stamp,rotation,position,full_relative=None,full_corners=None):
        if not sensors.SETTINGS.get('joint_rim'):
            return super().observe_partial(frame,stamp,rotation,position,full_relative,full_corners)
        h=self.inertial.history
        if not sensors.SETTINGS.get('partial_gate') or self.locked is None or h is None:return False
        if self.gate_number not in h.filter.gates or h.filter.rim is None:return False
        top=outer_top(frame.bgr)
        if top is None:return False
        if h.filter.localization:
            prediction=h.filter.measurement(self.gate_number,None,rim=True)
            if prediction is None:return False
            expected=prediction[0];width=np.linalg.norm(expected[1]-expected[0]);observed=np.linalg.norm(top[0][1]-top[0][0])
            if width<4. or not .65<observed/width<1.5 or np.linalg.norm(np.mean(top[0],axis=0)-expected.mean(axis=0))>width:return False
        if full_corners is not None and same_red_component(frame.bgr,full_corners,top[0]):
            self.association_counts['same_component_full_preferred']+=1;return False
        # The filter projects its learned shape. A partial view need not supply a metric pose.
        template=dict(width_m=3.,center_to_top_m=1.5,fractional_spread=1.,samples=0,seed_only=True)
        self.record_visual(frame,stamp,'outer_top',top[0],edge_residual_px=top[1],template=template)
        self.partial_observation=dict(gate_number=self.gate_number,image_id=frame.id,capture_time=stamp,
            top_corners=top[0].tolist(),edge_residual_px=top[1],template=template,
            position_relative_gate=(-self.gate_relative()).tolist(),metric_source='joint posterior; observation is pixels only',velocity=None)
        self.association_counts['partial_updates']+=1
        return True

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
            if sensors.SETTINGS.get('joint_rim') and not sensors.SETTINGS.get('map_localization',False):
                observed=full_rim_observation(frame.bgr,self.last_corners,pose)
                if observed is not None:
                    self.used_joint.add((frame.id,'top',self.gate_number))
                    self.inertial.pixels(self.gate_number,stamp,frame.id,*observed)
        if partial is not None and partial['image_id']==frame.id and partial['gate_number']==self.gate_number and (frame.id,'top',self.gate_number) not in self.used_joint:
            self.used_joint.add((frame.id,'top',self.gate_number));self.inertial.pixels(self.gate_number,stamp,frame.id,partial['top_corners'],partial['template'])
            partial['velocity']=dict(time=self.inertial.time,velocity=self.inertial.velocity.tolist(),source='joint posterior')
        self.fuse_lookahead(frame,stamp)
        f=self.inertial.history.filter
        if self.gate_number not in f.last_visual or (f.localization and self.inertial.time-f.last_visual[self.gate_number]>2.):
            if f.localization:
                if self.reacquire_since is None:self.reacquire_since=self.inertial.time
                self.acquire_started=self.reacquire_since
            self.locked=None
            if self.phase is not sensors.Phase.SHIFTING:self.phase=sensors.Phase.ACQUIRING
        elif f.last_visual[self.gate_number]==stamp:self.reacquire_since=None
        if self.gate_number in f.gates:
            self.anchor=f.gates[self.gate_number][1].copy();self.last_relative_gate=self.gate_relative();self.last_relative_position=f.p.copy()
            self.full_image_time=f.last_full_visual.get(self.gate_number)

    def drive(self,target):
        # Lift clear of the starting pad before asking for a large pitch/roll change.
        if self.phase is sensors.Phase.SHIFTING and not self.launch_cleared:
            h=self.inertial.history if hasattr(self.inertial,'history') else None
            self.launch_cleared=bool(self.inertial.position[2]<-.2 and (h is None or h.filter.airborne))
            self.signal['launch_cleared']=self.launch_cleared
            if not self.launch_cleared:
                request=sensors.acceleration_control(self.config,[0.,0.,-3.],self.inertial.rotation.to_euler(),0.)
                return BodyRates(0.,0.,0.,request.thrust)
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
            posterior=dict(position_covariance=f.P[:3,:3].tolist(),velocity_covariance=f.P[3:6,3:6].tolist(),attitude_covariance=f.P[6:9,6:9].tolist(),
                gyro_bias=f.bg.tolist(),force_bias=f.ba.tolist(),aperture_size_m=f.size,retargets=f.retargets,airborne=f.airborne,
                rim_dimensions=None if f.rim is None else f.rim[1].tolist())
            if f.upright_attitude:posterior['gravity_image_time']=float(f.last_gravity_time) if np.isfinite(f.last_gravity_time) else None
            if self.gate.gate_number in f.gates:
                relative,cov,camera,camcov=f.relative(self.gate.gate_number)
                posterior.update(relative_gate=relative.tolist(),relative_covariance=cov.tolist(),camera_gate=camera.tolist(),camera_covariance=camcov.tolist())
                posterior.update(gate_yaw=f.gates[self.gate.gate_number][2],last_accepted_image_time=f.last_visual.get(self.gate.gate_number),
                    position_gate_covariance=f.P[:3,f.gates[self.gate.gate_number][0]:f.gates[self.gate.gate_number][0]+3].tolist())
            self.gate.signal['joint_posterior']=posterior
            self.gate.signal['brake_test_state']=self.gate.brake_state
            if sensors.SETTINGS.get('command_epoch')!='public_send':
                self.gate.inertial.command(h.time,command.outputs);self.gate.inertial.sync()
            if self.gate.inertial.record is not None:
                self.gate.inertial.record(event='estimator_state',sequence=self.gate.inertial.input_count,state=estimator_state(h))
        return command

    def sent(self,command,stamp):
        self.gate.inertial.command(stamp,command.outputs)
