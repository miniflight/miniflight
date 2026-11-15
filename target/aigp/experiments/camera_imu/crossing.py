"""Gate-relative crossing constraints, using the pilot's estimated state only."""
import numpy as np


def predict(relative,velocity,direction):
    relative=np.asarray(relative);velocity=np.asarray(velocity);direction=np.asarray(direction)
    right=np.array([-direction[1],direction[0],0.])
    distance=float(relative @ direction);speed=float(velocity @ direction)
    seconds=distance/speed if distance>0. and speed>.5 else None
    projected=seconds is not None and seconds<=2.
    offset=relative-velocity*seconds if projected else relative
    return dict(range_m=distance,forward_speed_m_s=speed,time_to_approach_plane_s=seconds,
        lateral_position_m=float(relative @ right),height_position_m=float(relative[2]),
        lateral_velocity_m_s=float(velocity @ right),vertical_velocity_m_s=float(velocity[2]),
        predicted_width_m=float(offset @ right),predicted_down_m=float(offset[2]),constant_velocity_projection=projected)


class CrossingGuard:
    # Nominal1.5m opening, with a0.20m centerline reserve. Body clearance is uncalibrated.
    margin=.55

    def __init__(self):
        self.gate=None;self.active=False;self.steady=None

    def ready(self,relative,velocity,direction):
        p=predict(relative,velocity,direction)
        return not self.active and max(abs(p['predicted_width_m']),abs(p['predicted_down_m']))<=self.margin

    def update(self,gate,stamp,relative,velocity,direction,measurement_age):
        if gate!=self.gate:self.gate=gate;self.active=False;self.steady=None
        p=predict(relative,velocity,direction)
        braking=max(0.,p['forward_speed_m_s'])**2/(2*3.)+3.
        near=0.<p['range_m']<max(10.,braking)
        if near and max(abs(p['predicted_width_m']),abs(p['predicted_down_m']))>self.margin:self.active=True
        correction=None
        if self.active:
            settled=(max(abs(p['lateral_position_m']),abs(p['height_position_m']))<.25
                and np.hypot(p['lateral_velocity_m_s'],p['vertical_velocity_m_s'])<.3 and measurement_age<.3)
            self.steady=(stamp if self.steady is None else self.steady) if settled else None
            if self.steady is not None and stamp-self.steady>=.2:self.active=False
            else:correction=np.asarray(relative)-p['range_m']*np.asarray(direction)
        p.update(alignment_active=self.active,centerline_limit_m=self.margin,
            plane='pilot approach direction; not a native gate-plane measurement',posterior_margin_probability=None)
        return p,correction
