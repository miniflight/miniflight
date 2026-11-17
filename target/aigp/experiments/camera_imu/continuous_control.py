"""Continuous p/v/a gate tracking from the current public estimator map only."""
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from miniflight.gate_path import GateCurve,SpeedReference,unit,crossing_direction
from miniflight.position import trajectory_acceleration,quadratic_resistance,acceleration_control
from miniflight.control import BodyRates
from . import gate_filter_live as joint,sensors


class GateController(joint.GateController):
    def __init__(self,calls):
        super().__init__(calls)
        self.curve=None;self.plan_key=None;self.plan_index=None;self.future_index=None
        self.last_reference=None;self.initial_start=None;self.speed_reference=None;self.last_drive=None;self.replanned=-np.inf
        self.entry_speed=None
        self.recovery_stop=None
        self.recovery_released=-np.inf
        self.tracking_config=replace(self.config,max_acceleration=5.,max_horizontal_acceleration=9.,max_tilt=.9)

    def final_approach_ready(self):
        if not sensors.SETTINGS.get('view_recovery',False):return True
        if self.recovery_stop is not None:return False
        return self.inertial.time-self.inertial.history.filter.last_full_visual.get(self.gate_number,-np.inf)<.3

    def update(self,telemetry,frames,race_state):
        if sensors.SETTINGS.get('view_recovery',False) and self.recovery_stop is not None and self.phase is sensors.Phase.FINAL_IMU:self.phase=sensors.Phase.TRACKING
        return super().update(telemetry,frames,race_state)

    def end_direction(self,f,center):
        previous=f.gates.get(self.gate_number-1);future=f.gates.get(self.gate_number+1)
        before=previous[1] if previous is not None else center+(self.initial_start if self.initial_start is not None else self.inertial.position-center)
        direction=crossing_direction(before,center,None if future is None else future[1])
        return direction if direction @ self.direction>.2 else unit(np.r_[self.direction[:2],0.])

    def build(self,f,center,index,direction):
        start=self.inertial.position-center;start_direction=unit(-start);curvature=None
        if self.last_reference is not None:
            old=next((g for i,g,yaw in f.gates.values() if i==self.plan_index),None)
            if old is not None:
                r,v,a,at=self.last_reference;start=r+old-center
                if np.linalg.norm(v)>.1:
                    start_direction=unit(v);curvature=(a-at*start_direction)/np.linalg.norm(v)**2
        if np.linalg.norm(start)<.02:start=-.05*direction;curvature=None
        if self.speed_reference is None:self.speed_reference=SpeedReference(max(0.,float(self.inertial.velocity @ start_direction)))
        if self.plan_key!=self.gate_number or self.plan_index!=index:self.initial_start=start.copy()
        self.curve=GateCurve(start,start_direction,direction,curvature)
        self.plan_key=self.gate_number;self.plan_index=index;self.replanned=self.inertial.time
        future=f.gates.get(self.gate_number+1);self.future_index=None if future is None else future[0];self.entry_speed=None
        if future is not None:
            next_delta=future[1]-center;following=f.gates.get(self.gate_number+2)
            next_end=crossing_direction(center,future[1],None if following is None else following[1])
            if np.linalg.norm(next_delta)>.1:
                preview=GateCurve(-next_delta,direction,next_end)
                self.entry_speed=.95*preview.speed_limit(0.,self.config.max_speed)

    def drive(self,target):
        now=self.inertial.time;dt=0. if self.last_drive is None else max(0.,now-self.last_drive);self.last_drive=now
        h=self.inertial.history
        mapped_acquisition=self.phase is sensors.Phase.ACQUIRING and sensors.SETTINGS.get('map_prior') is not None
        if (self.phase not in (sensors.Phase.TRACKING,sensors.Phase.FINAL_IMU) and not mapped_acquisition) or h is None or self.gate_number not in h.filter.gates:
            return super().drive(target)
        f=h.filter;index,center,_=f.gates[self.gate_number];relative=self.inertial.position-center
        if self.direction is None:self.direction=unit((center-self.inertial.position)*[1.,1.,0.])
        direction=self.end_direction(f,center);future=f.gates.get(self.gate_number+1);future_index=None if future is None else future[0]
        changed=self.curve is None or self.plan_key!=self.gate_number or self.plan_index!=index
        if not changed and self.gate_remaining()>5. and now-self.replanned>.5:
            changed=future_index!=self.future_index or direction @ self.curve.end_direction<np.cos(.05)
        if changed:self.build(f,center,index,direction)
        u=self.curve.closest(relative);reference,tangent,curvature,_=self.curve.geometry(u)
        cap=self.curve.speed_limit(u,self.config.max_speed)
        if self.entry_speed is not None:
            distance=max(0.,(1-u)*self.curve.length*.8-self.speed_reference.speed*1.5)
            cap=min(cap,np.sqrt(self.entry_speed**2+2*1.5*distance))
        speed,at=self.speed_reference.update(cap,dt)
        if u>.999 and relative @ self.curve.end_direction>0.:
            tangent=self.curve.end_direction;reference=tangent*float(relative @ tangent);curvature=np.zeros(3)
        velocity=tangent*speed;feedforward=tangent*at+curvature*speed*speed
        yaw=f.gates[self.gate_number][2];normal=np.array([np.cos(yaw),np.sin(yaw),0.])
        if normal @ self.direction<0:normal=-normal
        remaining=-float(relative @ normal);side=relative+(remaining*normal)
        if sensors.SETTINGS.get('view_recovery',False):
            corners=np.array([[0.,-.75,-.75],[0.,.75,-.75],[0.,.75,.75],[0.,-.75,.75]]) @ Rotation.from_euler('z',yaw).as_matrix().T
            self.final_distance_m=2.5
            for distance in np.arange(1.5,8.1,.25):
                camera=(corners+distance*normal) @ f.R @ f.C.T;pixel=camera @ f.K.T
                if np.min(camera[:,2])<=.1:continue
                pixel=pixel[:,:2]/pixel[:,2,None]
                if np.all(pixel>[20.,20.]) and np.all(pixel<[620.,340.]):
                    self.final_distance_m=max(2.5,min(8.,distance+max(0.,float(self.inertial.velocity @ normal))*.3+.5));break
        full_time=f.last_full_visual.get(self.gate_number)
        if full_time is None:full_time=self.acquire_started if self.acquire_started is not None else now
        age=now-full_time;forward=float(self.inertial.velocity @ direction)
        braking=3.+max(0.,forward)**2/4.
        unsafe=sensors.SETTINGS.get('view_recovery',False) and self.phase is not sensors.Phase.FINAL_IMU and remaining>2. and np.isfinite(age) and (age>.35 or (now-self.recovery_released>.75 and remaining<max(8.,braking) and np.linalg.norm(side)>.5))
        if self.recovery_stop is None and unsafe:
            self.recovery_stop=float(relative @ normal)
            self.recovery_budget_s=min(8.,max(4.5,age+max(0.,remaining-18.)/3.+2.))
        recovering=self.recovery_stop is not None
        if recovering:
            reference=normal*float(relative @ normal);velocity=normal*min(3.,max(0.,remaining-8.));feedforward=np.zeros(3)
            bearing=self.hint_bearing()
            if age>.35 and bearing is not None:
                reference[2]=relative[2]+float(np.clip(8.*(bearing+.1),-3.,3.))
            self.speed_reference=SpeedReference(0.)
            settled=age<.3 and np.linalg.norm(side)<.4 and np.linalg.norm(self.inertial.velocity)<2.
            if settled:
                self.recovery_stop=None;self.curve=None;self.last_reference=None;self.recovery_released=now
        acceleration,_=trajectory_acceleration(relative,self.inertial.velocity,(reference,velocity,feedforward),(0.,0.,0.),dt,
            kp=self.config.position_gain*self.config.velocity_gain,ki=0.,kd=self.config.velocity_gain,integral_limit=0.)
        acceleration=np.array(acceleration);acceleration[:2]*=min(1.,4.5/max(np.linalg.norm(acceleration[:2]),1e-9));acceleration[2]=np.clip(acceleration[2],-3.,3.)
        R=self.inertial.rotation.as_rotation_matrix();drag=R @ quadratic_resistance(R.T @ self.inertial.velocity,self.config.body_resistance_per_m)
        compensation=acceleration+drag-self.inertial.prediction_bias;compensation[2]=np.clip(compensation[2],-5.,5.)
        a,b=self.config.thrust_curve;maximum=a*.325+b*.325**2;up=min(maximum,9.81-compensation[2]);compensation[2]=9.81-up
        horizontal=np.sqrt(max(0.,maximum*maximum-up*up));compensation[:2]*=min(1.,horizontal/max(np.linalg.norm(compensation[:2]),1e-9))
        request=acceleration_control(self.tracking_config,compensation,self.inertial.rotation.to_euler(),0.)
        thrust=float(np.clip(request.thrust,.21,.325))
        self.alignment_active=recovering;self.signal.update(visibility_alignment=recovering,crossing_alignment=False)
        self.signal['reference']=dict(gate=self.gate_number,time=float(now),position_relative_gate=reference.tolist(),velocity=velocity.tolist(),acceleration=feedforward.tolist(),
            parameter=float(u),speed_m_s=float(speed),target_speed_m_s=float(cap),tangential_acceleration_m_s2=float(at),replanned=bool(changed),
            position_error_m=float(np.linalg.norm(reference-relative)),force_limit_active=bool(abs(thrust-request.thrust)>1e-9),maximum_balanced_command=.325,recovery=recovering,measurement_age_s=float(age))
        self.last_reference=(reference.copy(),velocity.copy(),feedforward.copy(),at)
        return BodyRates(request.roll_rate,request.pitch_rate,request.yaw_rate,thrust)


class Controller(joint.Controller):
    def __init__(self,calls):
        self.gate=GateController(calls);self.motor=sensors.Motors(position=self.gate)
        self.timing=self.gate.timing;self.motor.timing=self.timing
