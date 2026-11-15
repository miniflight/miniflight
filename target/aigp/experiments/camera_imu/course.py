"""Course pilot using camera and public HIGHRES_IMU; native state is scorer-only."""
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np
import cv2

from miniflight import vehicle
from miniflight.loop import run
from miniflight.state import Frame, ImuClock, ImuIncrement
from miniflight.position import quadratic_resistance
from target.aigp.client import HeartbeatCadence
from target.aigp.experiments.recording import ProcessClient, RaceReader, RecordedController
from target.aigp.simulator import AIGPSimulator, ArmPhase
from . import controller as base


def vertical_bearing(pixel,orientation):
    ray=orientation @ (base.vision.K_INV @ np.r_[pixel,1.])
    return float(ray[2]/max(np.linalg.norm(ray[:2]),1e-6))


def alignment_step(predicted_height,age,bearing,direction):
    """A partial image supplies a steering cue, not a fabricated metric pose."""
    if age<=.3:return predicted_height
    if bearing is None:return 2.*direction
    # Keep the partial rim above the horizon until the full aperture returns.
    return float(np.clip(8.*(bearing+.1),-3.,3.))


def command_specific_force(config,outputs,body,velocity):
    """Predict force from our own commands and calibrated vehicle parameters."""
    if outputs is None:
        return body.T @ np.array([0.,0.,-9.81])
    motors=np.asarray(outputs)
    if config.thrust_curve is not None:
        linear,quadratic=config.thrust_curve
        thrust=float(np.mean(linear*motors+quadratic*motors**2))
    else:
        thrust=9.81+config.thrust_acceleration*(float(np.mean(motors))-config.hover_thrust)
    if config.body_resistance_per_m is not None:
        drag=np.array(quadratic_resistance(body.T @ velocity,config.body_resistance_per_m))
    else:
        drag=body.T @ quadratic_resistance(velocity,config.resistance_per_m)
    return np.array([0.,0.,-thrust])-drag


class WireClient(ProcessClient):
    """Map sensor clocks from public timestamps/arrivals; never read a native tap."""
    def __init__(self,calls,**kwargs):
        super().__init__(calls,**kwargs)
        self.imu_offsets,self.camera_offsets=deque(maxlen=256),deque(maxlen=128)
        self.last_imu_time,self.latest_frame=None,None
        self.filtered=0

    def _receive(self,message,now):
        if message.get_type() not in base.ALLOWED:
            return None
        result=super()._receive(message,now)
        if message.get_type()=='HIGHRES_IMU' and result is not None:
            self.imu_offsets.append(now-result.time)
        return result

    def poll(self,timeout=0.):
        output=[]
        for packet in super().poll(timeout):
            if not isinstance(packet.data,bytes) and packet.data.get_type() not in base.ALLOWED:
                self.filtered+=1
                continue
            if isinstance(packet.decoded,Frame):
                frame=self.align_frame(packet.decoded)
                if frame is None:
                    continue
                self.latest_frame=frame
                packet=replace(packet,decoded=self.latest_frame)
            output.append(packet)
        return tuple(output)

    def align_frame(self,frame):
        self.camera_offsets.append(frame.received_at-frame.time_ns*1e-9)
        if not self.imu_offsets:
            return None
        stamp=frame.time_ns*1e-9+min(self.camera_offsets)-min(self.imu_offsets)
        return replace(frame,time_ns=round(stamp*1e9))

    def read(self,timeout=1.):
        state=super().read(timeout)
        increments=[]
        for sample in state.imu_samples:
            if self.last_imu_time is not None and sample.time>self.last_imu_time:
                increments.append(ImuIncrement(sample.time,sample.time-self.last_imu_time,sample.acceleration,sample.gyro))
            self.last_imu_time=sample.time
        clock=None if state.time is None else ImuClock(state.time,state.received_at)
        self.state=replace(state,frame=self.latest_frame,imu_increments=tuple(increments),imu_increment_clock=clock)
        assert self.state.motion is None and self.state.attitude is None and self.state.motors is None
        return self.state


class GateController(base.GateController):
    def __init__(self,calls):
        super().__init__()
        self.calls=calls
        self.search_direction=np.array([1.,0.,0.])
        self.search_hint=None
        self.alignment_active=False
        self.alignment_settled=None
        self.alignment_gate=1
        self.alignment_direction=1.
        self.motor_outputs=None

    def drive(self,target):
        if self.alignment_gate!=self.gate_number:
            self.alignment_active=False;self.alignment_settled=None;self.alignment_gate=self.gate_number
            self.search_hint=None
        if self.gate_number>1 and self.phase is base.single.Phase.TRACKING and self.last_relative_gate is not None:
            age=self.inertial.time-self.last_image_time
            relative=self.gate_relative()
            if self.alignment_active or (abs(relative[2])>base.SETTINGS.get('visibility_height_change_m',1.5) and self.gate_remaining()>8.):
                if not self.alignment_active:
                    self.alignment_direction=float(np.sign(relative[2]))
                self.alignment_active=True
                visual=None if self.camera_observation is None else self.camera_observation.get('velocity')
                settled=(abs(relative[2])<base.SETTINGS.get('visibility_height_change_m',1.5) and abs(self.inertial.velocity[2])<.5 and age<.25
                         and visual is not None and abs(visual['velocity'][2])<.5)
                self.alignment_settled=(self.alignment_settled or self.inertial.time) if settled else None
                if self.alignment_settled is not None and self.inertial.time-self.alignment_settled>.2:
                    self.alignment_active=False
                else:
                    self.signal.update(visibility_alignment=True,alignment_relative_gate=relative.tolist(),
                        alignment_image_age_s=age,alignment_hint=self.search_hint)
                    if age>base.SETTINGS['maximum_blind_s']:
                        raise TimeoutError('height alignment could not regain the mapped gate')
                    correction=alignment_step(relative[2],age,self.hint_bearing(),self.alignment_direction)
                    horizontal_speed=np.linalg.norm(self.inertial.velocity[:2])
                    braking=8.+horizontal_speed**2/(2*self.config.max_horizontal_acceleration)
                    advance=base.SETTINGS['height_alignment_distance_m'] if self.gate_remaining()>braking else 0.
                    target=self.inertial.position+advance*self.direction
                    target[2]=self.inertial.position[2]+correction
                    self.signal.update(target=target.tolist(),alignment_height_step=correction)
                    self.signal['visibility_alignment']=True
                    self.signal['alignment_relative_gate']=relative.tolist()
                    return base.single.GateController.drive(self,target)
        self.signal['visibility_alignment']=False
        return super().drive(target)

    def hint_bearing(self):
        hint=self.search_hint
        if hint is None or self.inertial.time-hint['time']>.6:return None
        pose=self.inertial.camera_pose(min(self.inertial.time,hint['time']))
        return None if pose is None else vertical_bearing(hint['pixel'],pose[1])

    def search_waypoint(self,frames):
        speed=np.linalg.norm(self.inertial.velocity[:2])
        if speed>.5:
            self.search_direction=np.r_[self.inertial.velocity[:2]/speed,0.]
        if frames:
            frame=max(frames,key=lambda f:f.id)
            image=frame.bgr.astype(np.float32)/255
            blue,green,red=cv2.split(image)
            mask=np.uint8(red-(green+blue)/2>.08)
            count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
            candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=8 and 3<=stats[i,cv2.CC_STAT_WIDTH]<=180]
            if candidates:
                index=max(candidates,key=lambda i:stats[i,cv2.CC_STAT_AREA])
                self.search_hint=dict(pixel=centers[index].tolist(),area=int(stats[index,cv2.CC_STAT_AREA]),
                    time=frame.time_ns*1e-9,frame_id=frame.id)
        target=self.inertial.position+self.search_direction
        bearing=self.hint_bearing()
        target[2]+=alignment_step(0.,1.,.5 if bearing is None else bearing,1.)
        return target

    def update(self,telemetry,frames,race_state):
        if self.alignment_active:
            self.search_waypoint(frames)
        if self.gate_number>1 and self.phase is base.single.Phase.ACQUIRING and self.acquisition_prior is None:
            self.handoff_target=self.search_waypoint(frames)
        request=super().update(telemetry,frames,None)
        if self.locked is not None and base.SETTINGS.get('motion_prediction')=='command_model' and self.inertial.force_model is None:
            self.inertial.force_model=lambda body,velocity:command_specific_force(self.config,self.motor_outputs,body,velocity)
        if self.gate_number>1 and self.phase is base.single.Phase.ACQUIRING:
            self.signal['search_hint']=self.search_hint
        if self.gate_number==base.SETTINGS['gates_to_pass'] and self.locked is not None:
            # Extrapolate to command time, including ordinary gaps between IMU packets.
            age=max(0.,min(.3,self.calls.monotonic()-vehicle.state.received_at))
            position=self.inertial.position+self.inertial.velocity*age
            if self.gate_remaining()-(self.inertial.velocity @ self.direction)*age < 0.:
                self.phase=base.single.Phase.COMPLETE
                self.completed.append(dict(gate=self.gate_number,imu_time=self.inertial.time,
                    estimated_center=self.anchor.tolist(),position=position.tolist(),velocity=self.inertial.velocity.tolist(),
                    image_id=self.last_image_id,image_time=self.last_image_time))
                self.signal.update(estimated_gates_passed=len(self.completed),completed_estimates=self.completed)
                raise StopIteration
        return request


class Controller(base.single.BaseController):
    targets=('vq1.r1',)
    def __init__(self,calls):
        self.gate=GateController(calls)
        self.motor=base.single.Motors(position=self.gate)

    def update(self,telemetry,frames,race_state):
        command=self.motor.update(telemetry,frames,None)
        if command is not None:
            self.gate.motor_outputs=command.outputs
            self.gate.signal.update(gyro=self.motor.gyro,rate_request=self.motor.rate_request,rate_integral=self.motor.integral)
            self.gate.signal['visual_gravity']=self.gate.gravity_observation
        return command


def execute(calls,record,path,reader=None,client=None):
    """Own sensor/command lifetime; native GO, validity, counters and finish are unused."""
    client=WireClient(calls,record_camera=True) if client is None else client
    controller=RecordedController(lambda:Controller(calls),record,signals=lambda c:c.gate.signal)
    cleanup=AIGPSimulator(Controller,client=client,clock=calls)
    score_reader=RaceReader(calls,reader)
    began=calls.monotonic();ready=None;error=None;reason=None;armed_at=None
    heartbeat=HeartbeatCadence()
    vehicle.bind(client)
    vehicle.connect()
    def read_signals(now):
        nonlocal ready,reason
        state=vehicle.read(timeout=0)
        score_reader.poll()  # Recorded for scoring; no returned value enters navigation or lifecycle.
        heartbeat.send_if_due(client,now)
        if client.connected and ready is None:
            ready=now+base.SETTINGS.get('startup_allowance_s',8.)
            controller()
        if now-began>base.SETTINGS.get('course_timeout_s',180.):
            raise TimeoutError('sensor-only course deadline exceeded')
        if ready is None or now<ready:
            return None
        feedback=state.rate_feedback
        if cleanup.arm_phase is ArmPhase.ARM_REQUESTED and (feedback is None or now-feedback.received_at>.3):
            reason='IMU stream stopped'
            raise StopIteration
        telemetry=tuple(p for p in client.packets if not isinstance(p.data,bytes))
        frames=tuple(p.decoded for p in client.packets if isinstance(p.decoded,Frame))
        return telemetry,frames,None
    def send(command):
        nonlocal armed_at
        if command is None:
            return
        if calls.monotonic()-vehicle.state.received_at>.3:
            raise TimeoutError('sensor-only command exceeded IMU age limit')
        if cleanup.arm_phase is ArmPhase.IDLE:
            vehicle.validate(command);vehicle.arm();cleanup.arm_phase=ArmPhase.ARM_REQUESTED
            armed_at=vehicle.state.time
        vehicle.send(command)
    try:
        run(read_signals,controller.update,send,hz=50.,clock=calls)
    except StopIteration:
        reason=reason or 'estimated course complete'
    except Exception as exception:
        error=dict(type=type(exception).__name__,message=str(exception))
        record(event='flight_error',error=error)
    finally:
        if calls.record is not None and client.latest_frame is not None:
            cv2.imwrite(str(Path(path).parent/'last-camera.jpg'),client.latest_frame.bgr)
        try:
            cleanup.stop()
        finally:
            score_reader.poll()
            vehicle.disconnect()
    instance=controller.controller
    outcome=dict(error=error,connection_closed=not client.connected,filtered_packets=client.filtered,
        phase=None if instance is None else instance.gate.phase.name,
        final_signal=None if instance is None else instance.gate.signal,race=None,termination=reason,
        estimated_gates_passed=0 if instance is None else len(instance.gate.completed),
        estimated_gate_number=0 if instance is None else instance.gate.gate_number,armed_boot_time=armed_at)
    if instance is not None:
        outcome['prediction_bias']=instance.gate.inertial.prediction_bias.tolist()
    record(event='result',**outcome)
    return outcome,client
