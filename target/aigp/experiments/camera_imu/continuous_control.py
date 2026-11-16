"""Continuous p/v/a gate tracking from the current public estimator map only."""
from dataclasses import replace
import numpy as np
from miniflight.gate_path import GateCurve,SpeedReference,unit
from miniflight.position import trajectory_acceleration,quadratic_resistance,acceleration_control
from miniflight.control import BodyRates
from . import gate_filter_live as joint,sensors


class GateController(joint.GateController):
    def __init__(self,calls):
        super().__init__(calls)
        self.curve=None;self.plan_key=None;self.plan_index=None;self.future_index=None
        self.last_reference=None;self.initial_start=None;self.speed_reference=None;self.last_drive=None;self.replanned=-np.inf
        self.entry_speed=None
        self.tracking_config=replace(self.config,max_acceleration=5.,max_horizontal_acceleration=9.,max_tilt=.9)

    def final_approach_ready(self):return True

    def end_direction(self,f,center):
        previous=f.gates.get(self.gate_number-1);future=f.gates.get(self.gate_number+1)
        before=previous[1] if previous is not None else center+(self.initial_start if self.initial_start is not None else self.inertial.position-center)
        delta=(future[1] if future is not None else center)-before
        direction=unit(delta)
        return direction if direction @ self.direction>.2 else unit(self.direction)

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
            next_end=unit(next_delta if following is None else following[1]-center)
            if np.linalg.norm(next_delta)>.1:
                preview=GateCurve(-next_delta,direction,next_end)
                self.entry_speed=.95*preview.speed_limit(0.,self.config.max_speed)

    def drive(self,target):
        now=self.inertial.time;dt=0. if self.last_drive is None else max(0.,now-self.last_drive);self.last_drive=now
        h=self.inertial.history
        if self.phase not in (sensors.Phase.TRACKING,sensors.Phase.FINAL_IMU) or h is None or self.gate_number not in h.filter.gates:
            return super().drive(target)
        f=h.filter;index,center,_=f.gates[self.gate_number];relative=self.inertial.position-center
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
        acceleration,_=trajectory_acceleration(relative,self.inertial.velocity,(reference,velocity,feedforward),(0.,0.,0.),dt,
            kp=self.config.position_gain*self.config.velocity_gain,ki=0.,kd=self.config.velocity_gain,integral_limit=0.)
        acceleration=np.array(acceleration);acceleration[:2]*=min(1.,4.5/max(np.linalg.norm(acceleration[:2]),1e-9));acceleration[2]=np.clip(acceleration[2],-3.,3.)
        R=self.inertial.rotation.as_rotation_matrix();drag=R @ quadratic_resistance(R.T @ self.inertial.velocity,self.config.body_resistance_per_m)
        compensation=acceleration+drag;compensation[2]=np.clip(compensation[2],-5.,5.)
        a,b=self.config.thrust_curve;maximum=a*.325+b*.325**2;up=min(maximum,9.81-compensation[2]);compensation[2]=9.81-up
        horizontal=np.sqrt(max(0.,maximum*maximum-up*up));compensation[:2]*=min(1.,horizontal/max(np.linalg.norm(compensation[:2]),1e-9))
        request=acceleration_control(self.tracking_config,compensation,self.inertial.rotation.to_euler(),0.)
        thrust=float(np.clip(request.thrust,.21,.325))
        self.alignment_active=False;self.signal.update(visibility_alignment=False,crossing_alignment=False)
        self.signal['reference']=dict(gate=self.gate_number,time=float(now),position_relative_gate=reference.tolist(),velocity=velocity.tolist(),acceleration=feedforward.tolist(),
            parameter=float(u),speed_m_s=float(speed),target_speed_m_s=float(cap),tangential_acceleration_m_s2=float(at),replanned=bool(changed),
            position_error_m=float(np.linalg.norm(reference-relative)),force_limit_active=bool(abs(thrust-request.thrust)>1e-9),maximum_balanced_command=.325)
        self.last_reference=(reference.copy(),velocity.copy(),feedforward.copy(),at)
        return BodyRates(request.roll_rate,request.pitch_rate,request.yaw_rate,thrust)


class Controller(joint.Controller):
    def __init__(self,calls):
        self.gate=GateController(calls);self.motor=sensors.Motors(position=self.gate)
        self.timing=self.gate.timing;self.motor.timing=self.timing
