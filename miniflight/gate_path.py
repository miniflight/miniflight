"""A spatial gate reference with bounded speed acceleration and jerk. SI units."""
import numpy as np


def unit(vector):
    vector=np.asarray(vector,float);return vector/max(np.linalg.norm(vector),1e-9)


def crossing_direction(previous,center,following=None):
    """Carry monotone height changes; level at missing lookahead or a reversal."""
    before=np.asarray(center,float)-previous;after=None if following is None else np.asarray(following,float)-center
    horizontal=(before if after is None else before+after).copy();horizontal[2]=0.
    slope=0.
    if after is not None:
        h0,h1=np.linalg.norm(before[:2]),np.linalg.norm(after[:2])
        if min(h0,h1)>1e-6:
            d0,d1=before[2]/h0,after[2]/h1
            if d0*d1>0:
                w0,w1=2*h1+h0,h1+2*h0;slope=(w0+w1)/(w0/d0+w1/d1)
    horizontal[2]=slope*np.linalg.norm(horizontal[:2])
    return unit(horizontal)


class GateCurve:
    def __init__(self,start,start_direction,end_direction,start_curvature=None):
        self.length=float(np.linalg.norm(start));self.end_direction=unit(end_direction)
        if self.length<1e-6:raise ValueError('A gate curve needs a nonzero chord')
        a=np.asarray(start,float);v0=self.length*unit(start_direction);v1=self.length*self.end_direction
        a0=np.zeros(3) if start_curvature is None else self.length**2*np.asarray(start_curvature)
        c2=.5*a0;d=-a-v0-c2;e=v1-v0-2*c2;f=-2*c2
        self.coefficients=np.array([a,v0,c2,10*d-4*e+.5*f,-15*d+7*e-f,6*d-3*e+.5*f])
        self.parameter=0.

    def derivatives(self,u):
        c=self.coefficients
        p=sum(c[k]*u**k for k in range(6))
        v=sum(k*c[k]*u**(k-1) for k in range(1,6));a=sum(k*(k-1)*c[k]*u**(k-2) for k in range(2,6))
        return p,v,a

    def geometry(self,u):
        p,v,a=self.derivatives(u);scale=max(1e-9,np.linalg.norm(v));tangent=v/scale
        curvature=(a-tangent*(tangent @ a))/scale**2
        return p,tangent,curvature,scale

    def closest(self,position):
        p=np.asarray(position);grid=np.linspace(max(0.,self.parameter-.15),1.,17)
        u=float(min(grid,key=lambda x:np.linalg.norm(self.derivatives(x)[0]-p)))
        for _ in range(6):
            q,v,a=self.derivatives(u);denominator=float(v @ v+(q-p) @ a)
            if denominator<=1e-8:break
            u=float(np.clip(u-((q-p) @ v)/denominator,0.,1.))
        self.parameter=u;return u

    def speed_limit(self,u,maximum,normal_acceleration=2.,vertical_speed=3.):
        limit=maximum
        for future in np.linspace(u,1.,13):
            _,tangent,curvature,scale=self.geometry(future)
            _,_,before,_=self.geometry(max(0.,future-.001));_,_,after,_=self.geometry(min(1.,future+.001))
            span=(min(1.,future+.001)-max(0.,future-.001))*scale
            change=np.linalg.norm(after-before)/max(span,1e-9)
            cap=min(maximum,np.sqrt(normal_acceleration/max(np.linalg.norm(curvature),1e-9)),
                vertical_speed/max(abs(tangent[2]),1e-9),(5./max(change,1e-9))**(1/3))
            distance=(future-u)*self.length*.8
            limit=min(limit,np.sqrt(cap*cap+2*1.5*distance))
        return max(.3,float(limit))


class SpeedReference:
    def __init__(self,speed=0.,acceleration=3.,jerk=6.):
        self.speed=float(speed);self.acceleration=0.;self.maximum_acceleration=acceleration;self.maximum_jerk=jerk

    def update(self,target,dt):
        if dt<0 or not np.isfinite(target+dt):raise ValueError('Reference time and target must be finite and causal')
        steps=max(1,int(np.ceil(dt/.02)));step=dt/steps
        for _ in range(steps):
            error=target-self.speed
            jerk=float(np.clip(4.*error-4.*self.acceleration,-self.maximum_jerk,self.maximum_jerk))
            new=float(np.clip(self.acceleration+jerk*step,-self.maximum_acceleration,self.maximum_acceleration))
            self.speed+=.5*(self.acceleration+new)*step;self.acceleration=new
        return self.speed,self.acceleration
