"""Course pilot using camera and public HIGHRES_IMU; native state is scorer-only."""
from collections import deque
from dataclasses import replace

import numpy as np

from miniflight import vehicle
from miniflight.loop import run
from miniflight.state import Frame, ImuClock, ImuIncrement
from target.aigp.client import HeartbeatCadence
from target.aigp.experiments.recording import ProcessClient, RaceReader, RecordedController
from target.aigp.simulator import AIGPSimulator, ArmPhase
from . import controller as base


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

    def update(self,telemetry,frames,race_state):
        request=super().update(telemetry,frames,None)
        if self.gate_number==base.SETTINGS['gates_to_pass'] and self.locked is not None:
            # Extrapolate to command time, including ordinary gaps between IMU packets.
            age=max(0.,min(.3,self.calls.monotonic()-vehicle.state.received_at))
            position=self.inertial.position+self.inertial.velocity*age
            if (self.anchor-position) @ self.direction < 0.:
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
        return self.motor.update(telemetry,frames,None)


def execute(calls,record,path,reader=None,client=None):
    """Own sensor/command lifetime; native GO, validity, counters and finish are unused."""
    client=WireClient(calls,record_camera=True) if client is None else client
    controller=RecordedController(lambda:Controller(calls),record,signals=lambda c:c.gate.signal)
    cleanup=AIGPSimulator(Controller,client=client,clock=calls)
    score_reader=RaceReader(calls,reader)
    began=calls.monotonic();ready=None;error=None;reason=None
    heartbeat=HeartbeatCadence()
    vehicle.bind(client)
    vehicle.connect()
    def read_signals(now):
        nonlocal ready,reason
        state=vehicle.read(timeout=0)
        score_reader.poll()  # Recorded for scoring; no returned value enters navigation or lifecycle.
        heartbeat.send_if_due(client,now)
        if client.connected and ready is None:
            ready=now+4.  # Fixed startup allowance after the acknowledged reset.
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
        if command is None:
            return
        if calls.monotonic()-vehicle.state.received_at>.3:
            raise TimeoutError('sensor-only command exceeded IMU age limit')
        if cleanup.arm_phase is ArmPhase.IDLE:
            vehicle.validate(command);vehicle.arm();cleanup.arm_phase=ArmPhase.ARM_REQUESTED
        vehicle.send(command)
    try:
        run(read_signals,controller.update,send,hz=50.,clock=calls)
    except StopIteration:
        reason=reason or 'estimated course complete'
    except Exception as exception:
        error=dict(type=type(exception).__name__,message=str(exception))
        record(event='flight_error',error=error)
    finally:
        try:
            cleanup.stop()
        finally:
            score_reader.poll()
            vehicle.disconnect()
    instance=controller.controller
    outcome=dict(error=error,connection_closed=not client.connected,filtered_packets=client.filtered,
        phase=None if instance is None else instance.gate.phase.name,
        final_signal=None if instance is None else instance.gate.signal,race=None,termination=reason)
    record(event='result',**outcome)
    return outcome,client
