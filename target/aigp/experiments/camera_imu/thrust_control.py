"""Short declared collective pulses in a camera/IMU position hold."""
import numpy as np
from miniflight.control import BodyRates
from . import gate_filter_live as joint,sensors


def sequence(kind):
    levels=[.21,.235,.26,.285,.31,.335]
    order=[2,5,0,4,1,3,5,1,4,0,3,2] if kind=='train' else [4,0,3,5,2,1,0,5,1,3,4,2]
    return [(levels[k],(.18 if i%2==0 else .30) if kind=='train' else (.24 if i%2==0 else .36)) for i,k in enumerate(order[:6])]


class GateController(joint.GateController):
    def __init__(self,calls):
        super().__init__(calls);self.hold=None;self.started=None;self.pulse_at=None;self.recover_at=None
        self.steady=None;self.index=0;self.steps=sequence(sensors.SETTINGS['thrust_excitation'])

    def drive(self,target):
        if self.phase is not sensors.Phase.TRACKING:return super().drive(target)
        now=self.inertial.time
        if self.hold is None:
            if self.gate_remaining()>10.:return super().drive(target)
            self.hold=self.inertial.position.copy();self.hold[2]=self.anchor[2]+.4;self.started=now
        if now-self.started>75.:raise TimeoutError('thrust excitation did not settle in75s')
        error=np.linalg.norm(self.inertial.position-self.hold);speed=np.linalg.norm(self.inertial.velocity)
        fresh=now-self.measurement_time()<.2
        settled=error<.2 and speed<.25 and fresh and np.linalg.norm(self.inertial.gyro)<.15
        self.steady=(now if self.steady is None else self.steady) if settled else None
        if self.pulse_at is None and self.steady is not None and now-self.steady>=.5:
            if self.index==len(self.steps):raise StopIteration('public thrust excitation complete')
            self.pulse_at=now;self.steady=None
        level,duration=self.steps[min(self.index,len(self.steps)-1)]
        if self.pulse_at is not None and now-self.pulse_at>=duration:
            self.index+=1;self.pulse_at=None;self.recover_at=now;self.steady=None
        active=self.pulse_at is not None
        coast=bool(self.recover_at is not None and now-self.recover_at<.4)
        applied=active and error<=1.5 and speed<=3. and fresh
        self.signal['excitation']=dict(index=self.index,phase='pulse' if active else 'coast' if coast else 'recover',applied=bool(applied),body_rates_held=active or coast,
            collective=level if active else None,duration_s=duration,hold_error_m=float(error),speed_m_s=float(speed),image_fresh=bool(fresh))
        self.signal['target']=self.hold.tolist()
        request=sensors.GateController.drive(self,self.hold)
        if applied:return BodyRates(0.,0.,0.,level)
        if coast:return BodyRates(0.,0.,0.,request.thrust)
        return request


class Controller(joint.Controller):
    def __init__(self,calls):
        self.gate=GateController(calls);self.motor=sensors.Motors(position=self.gate)
        self.timing=self.gate.timing;self.motor.timing=self.timing
