"""Independent public-sensor setup pilot, then a cold camera/map controller."""
from concurrent.futures import ThreadPoolExecutor
import json,subprocess,sys
import time
import numpy as np
from miniflight import vehicle
from miniflight.control import BodyRates
from miniflight.position import acceleration_control,quadratic_resistance
from miniflight.profiling import timed
from miniflight.relocalization import locate,bootstrap
from target.aigp.controllers.r1_motors import RatePhase
from . import continuous_control as continuous,gate_filter_live as joint,sensors,vision


def solve(prior,views,seed):
    wall=time.perf_counter_ns();payload=dict(prior=prior,views=views,seed=np.asarray(seed).tolist(),camera=vision.CAMERA_FROM_BODY.tolist(),intrinsics=vision.K.tolist())
    completed=subprocess.run([sys.executable,'-m','target.aigp.experiments.camera_imu.localize_worker'],input=json.dumps(payload),text=True,capture_output=True,check=True)
    data=json.loads(completed.stdout);data['profile']['wall_ns']=time.perf_counter_ns()-wall
    return data['result'],data['profile']


class ColdInertial(joint.Inertial):
    def __init__(self,config,record=None,calls=None):
        super().__init__(config,record);self.views=[];self.pending=[];self.seen=set();self.attempted=-np.inf;self.last_result=None
        self.calls=calls;self.pool=None;self.job=None;self.job_views=None;self.job_seed=None;self.applied_commands=[]
        self.primary=None

    def update(self,samples):
        if self.history is not None:return super().update(samples)
        for sample in samples:
            if self.seed and sample.time<=self.seed[-1].time:continue
            self.seed.append(sample)
        if self.seed:self.time=self.seed[-1].time;self.gyro=self.seed[-1].gyro

    @timed('relocalization')
    def acquire(self,frames):
        self.pending.extend(f for f in frames if f.id not in self.seen)
        for frame in self.pending:
            if frame.id in self.seen:continue
            stamp=frame.time_ns*1e-9
            if not self.seed or not self.seed[0].time<=stamp<=self.seed[-1].time:continue
            self.seen.add(frame.id);pixels=list(vision.aperture_candidates(frame.bgr,close_gaps=True,strict=True))
            if pixels:self.primary=(stamp,float(np.mean(pixels[0][:,1])))
            if self.record is not None:
                for p in pixels:self.record(event='visual_measurement',image_id=frame.id,capture_time=stamp,kind='localization_candidate',points=p.tolist(),context_gate=0,quality=dict(unassigned=True))
            if len(pixels)>=2:self.views.append(dict(time=stamp,image_id=frame.id,pixels=[p.tolist() for p in pixels]))
        self.pending=[f for f in self.pending if f.id not in self.seen and (not self.seed or f.time_ns*1e-9>=self.seed[0].time)]
        prior=sensors.SETTINGS['map_prior']
        if self.job is not None:
            ready=self.calls('localization_ready',dict(image_id=self.job_views[-1]['image_id']),self.job.done)
            if not ready:return False
            result,profile=self.job.result();self.job=None;views=self.job_views;seed=self.job_seed
            if self.record is not None:self.record(event='profile',scope='localization_solver',stages=dict(global_fit=profile))
        else:
            if not self.views or self.time-self.attempted<.3:return False
            self.attempted=self.time;available=[v for v in self.views if v['time']>=self.views[-1]['time']-3.]
            views=[available[i] for i in np.linspace(0,len(available)-1,min(12,len(available)),dtype=int)]
            seed=np.array([[s.time,*s.acceleration,*s.gyro] for s in self.seed if views[0]['time']-.15<=s.time<=views[-1]['time']+.03])
            if len(views)<3 or views[-1]['time']-views[0]['time']<.5:result=dict(accepted=False,reason='short_window')
            elif self.calls is None:result=locate(prior,views,seed,vision.CAMERA_FROM_BODY,vision.K)
            else:
                if self.pool is None:self.pool=ThreadPoolExecutor(max_workers=1)
                self.job_views=views;self.job_seed=seed;self.job=self.pool.submit(solve,prior,views,seed);return False
        self.last_result={k:v for k,v in result.items() if k not in ['rotation','hypotheses']}
        self.last_result.update(window_frames=len(views),window_span_s=views[-1]['time']-views[0]['time'],two_gate_frames=len(self.views),last_image_time=views[-1]['time'])
        if 'hypotheses' in result:self.last_result['alternatives']=[{k:h[k] for k in ['ids','score','rms_px']} for h in result['hypotheses']]
        if not result['accepted']:return False
        seed=np.array([[s.time,*s.acceleration,*s.gyro] for s in self.seed if s.time>=seed[0,0]])
        ordered=sorted(self.applied_commands,key=lambda r:r[0]);before=[r for r in ordered if r[0]<=result['time']]
        commands=before[-1:]+[r for r in ordered if r[0]>result['time']]
        c=dict(thrust_curve=list(self.config.thrust_curve),drag=list(self.config.body_resistance_per_m),camera_from_body=vision.CAMERA_FROM_BODY.tolist(),intrinsics=vision.K.tolist(),
            supported=False,attitude_corrections=not sensors.SETTINGS.get('gyro_attitude',False),broad_rim=sensors.SETTINGS.get('joint_rim',False),partial_gate_births=False,
            image_time_sigma_s=.01 if sensors.SETTINGS.get('image_time_uncertainty',False) else 0.,upright_attitude=sensors.SETTINGS.get('upright_attitude',False),history_s=2.,map_prior=prior,bootstrap_views=views,bootstrap_commands=commands)
        self.history,_=bootstrap(c,seed,views,result);self.bootstrap_image=views[-1]['image_id'];self.event('init',seed=seed.tolist(),config=c)
        self.times=[t for t,_ in self.history.snapshots];self.positions=[s.p.copy() for _,s in self.history.snapshots];self.velocities=[s.v.copy() for _,s in self.history.snapshots]
        self.rotations=[s.R @ vision.CAMERA_FROM_BODY.T for _,s in self.history.snapshots];self.sync()
        if self.pool is not None:self.pool.shutdown(wait=False)
        return True


class ColdGate(continuous.GateController):
    def __init__(self,calls):
        super().__init__(calls);self.inertial=ColdInertial(self.config,getattr(calls,'record',None),calls);self.inertial.timing=self.timing
        self.hold_thrust=self.config.hover_thrust;self.hold_time=None

    def update(self,telemetry,frames,race_state):
        if self.inertial.history is None:
            self.inertial.update(vehicle.state.imu_increments)
            ready=self.inertial.acquire(frames)
            self.signal=dict(phase='LOCALIZING',imu_time=vehicle.state.time,motion_available=False,attitude_available=False,localization=self.inertial.last_result)
            if not ready:
                now=self.inertial.time;dt=0. if self.hold_time is None else min(.05,now-self.hold_time);self.hold_time=now
                force=float(np.mean([np.linalg.norm(s.acceleration) for s in self.inertial.seed if s.time>=now-.1]))
                a,b=self.config.thrust_curve;slope=a+2*b*self.hold_thrust
                self.hold_thrust=float(np.clip(self.hold_thrust+2.*(9.81-force)*dt/slope,.21,.325))
                primary=self.inertial.primary;pitch=0. if primary is None or now-primary[0]>.2 else float(np.clip(-.5*(primary[1]-200.)/vision.K[1,1],-.08,.08))
                self.signal['localization_hold']=dict(thrust=self.hold_thrust,force_norm_m_s2=force,primary_y_px=None if primary is None else primary[1],pitch_rate_rad_s=pitch)
                return BodyRates(0.,pitch,0.,self.hold_thrust)
            self.phase=sensors.Phase.ACQUIRING;self.acquire_started=self.began=self.inertial.time;self.shift_displacement=np.zeros(3);self.launch_cleared=True
            for k in self.inertial.history.filter.last_visual:self.used_joint.add((self.inertial.bootstrap_image,'full',k))
        request=super().update(telemetry,frames,None);self.signal['localization']=self.inertial.last_result;return request


class ColdController(joint.Controller):
    def __init__(self,calls):
        self.gate=ColdGate(calls);self.motor=sensors.Motors(position=self.gate);self.timing=self.gate.timing;self.motor.timing=self.timing

    def sent(self,command,stamp):
        if self.gate.inertial.history is None:
            self.gate.inertial.applied_commands.append([float(stamp),*command.outputs])
        else:super().sent(command,stamp)


class SetupGate(continuous.GateController):
    def __init__(self,calls,fixture):
        super().__init__(calls);self.fixture=fixture;self.fixture_epoch=None;self.inertial.record=None

    def update(self,telemetry,frames,race_state):
        if self.phase is sensors.Phase.SHIFTING and self.launch_cleared and self.inertial.position[2]<-.4:
            self.phase=sensors.Phase.TRACKING;self.began=self.acquire_started=self.inertial.time;self.shift_displacement=np.zeros(3)
        return super().update(telemetry,frames,None)

    def drive(self,target):
        if self.fixture_epoch is None:self.fixture_epoch=self.inertial.time
        if not self.launch_cleared:return joint.GateController.drive(self,target)
        p,v=self.inertial.position,self.inertial.velocity;R=self.inertial.rotation.as_rotation_matrix()
        target=np.array(self.fixture['target']).copy()
        if p[0]<3.:target[2]=-.6
        wanted=sensors.velocity_target(target-p,self.config.position_gain,1.2,1.)
        acceleration=self.config.velocity_gain*(wanted-v)
        acceleration+=R @ quadratic_resistance(R.T @ v,self.config.body_resistance_per_m)-self.inertial.prediction_bias
        if self.inertial.time-self.fixture_epoch>=self.fixture['delay_s']:acceleration[:2]=0.
        request=acceleration_control(self.config,acceleration,self.inertial.rotation.to_euler(),self.fixture['yaw'])
        return BodyRates(request.roll_rate,request.pitch_rate,request.yaw_rate,float(np.clip(request.thrust,.21,.325)))


class Controller(joint.Controller):
    def __init__(self,calls,fixture):
        self.calls=calls;self.fixture=fixture;self.setup=joint.Controller(calls);self.setup.gate=SetupGate(calls,fixture)
        self.setup.motor=sensors.Motors(position=self.setup.gate);self.setup.timing=self.setup.gate.timing;self.setup.motor.timing=self.setup.timing
        self.cold=None;self.active=False;self.started=None;self.arm_ack=None

    @property
    def gate(self):return self.cold.gate if self.cold is not None else self.setup.gate
    @property
    def timing(self):return self.cold.timing if self.cold is not None else self.setup.timing
    @property
    def localizing(self):return self.cold is not None and self.cold.gate.inertial.history is None

    def update(self,telemetry,frames,race_state):
        for p in telemetry:
            if p.data.get_type()=='COMMAND_ACK' and p.data.command==400 and p.data.result==0:self.arm_ack=p.received_at
        if self.active:
            if self.localizing and vehicle.state.time-self.started>6.:raise TimeoutError('Cold map localization exceeded six seconds')
            return self.cold.update(telemetry,frames,None)
        command=self.setup.update(telemetry,frames,None)
        if command is not None:self.setup.gate.signal['phase']='SETUP'
        if self.cold is None and self.setup.gate.fixture_epoch is not None and self.setup.gate.inertial.time-self.setup.gate.fixture_epoch>=self.fixture['delay_s']:
            self.cold=ColdController(self.calls);self.started=vehicle.state.time
            if self.arm_ack is None:raise RuntimeError('Cold handoff needs the public arm acknowledgement')
            self.cold.motor.arm_received_at=self.arm_ack;self.cold.motor.rate_phase=RatePhase.WAITING_FOR_POST_ARM_SAMPLE
            self.active=True;self.setup=None;return self.cold.update(telemetry,frames,None)
        return command

    def sent(self,command,stamp):
        if self.active:self.cold.sent(command,stamp)
        else:
            self.setup.sent(command,stamp)
            if self.cold is not None:self.cold.gate.inertial.applied_commands.append([float(stamp),*command.outputs])
