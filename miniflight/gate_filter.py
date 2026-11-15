"""Joint gate/vehicle error-state filter. No simulator or transport imports."""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation


def skew(x):
    a,b,c=x
    return np.array([[0.,-c,b],[c,0.,-a],[-b,a,0.]])


class GateFilter:
    """Error coordinates: p,v,right attitude error,gyro bias,force bias,gate(xyz,yaw)."""
    def __init__(self,rotation,gyro_bias,thrust_curve,drag,camera_from_body,intrinsics):
        self.p=np.zeros(3);self.v=np.zeros(3);self.R=np.array(rotation)
        self.bg=np.array(gyro_bias);self.ba=np.zeros(3);self.gates={};self.rim=None;self.size=1.5
        self.P=np.diag(np.r_[np.full(3,1e-8),np.full(3,.05**2),np.r_[np.full(2,np.radians(.5)**2),1e-8],
            np.full(3,.001**2),np.full(3,.5**2),.03**2])
        self.thrust_curve=np.array(thrust_curve);self.drag=np.asarray(drag)
        self.C=np.array(camera_from_body);self.K=np.array(intrinsics)
        self.acceleration_noise=.5;self.gyro_noise=.001
        self.gyro_bias_walk=.0001;self.force_bias_walk=.05

    def copy(self):return deepcopy(self)

    def inject(self,dx):
        self.p+=dx[:3];self.v+=dx[3:6];self.R=self.R @ Rotation.from_rotvec(dx[6:9]).as_matrix()
        self.bg+=dx[9:12];self.ba+=dx[12:15]
        self.size+=dx[15]
        for key,(index,center,yaw) in list(self.gates.items()):
            self.gates[key]=(index,center+dx[index:index+3],yaw+dx[index+3])
        if self.rim is not None:
            index,dimensions=self.rim;self.rim=(index,dimensions+dx[index:index+2])

    def predict(self,dt,gyro,motors,angle_uncertainty=None):
        if not 0.<dt<=.15:raise ValueError('Process interval must be positive elapsed public time, <=150ms')
        n=len(self.P);F=np.eye(n);Q=np.zeros((n,n));omega=np.asarray(gyro)-self.bg
        rotation_step=Rotation.from_rotvec(omega*dt).as_matrix()
        if motors is None:
            # Before our first command, the disarmed stationary prior supplies support.
            acceleration=np.zeros(3);Av=np.zeros((3,3));At=np.zeros((3,3))
        else:
            u=np.asarray(motors);a,b=self.thrust_curve;thrust=np.mean(a*u+b*u*u)
            local=self.R.T @ self.v;D=np.diag(2*self.drag*abs(local))
            force=np.array([0.,0.,-thrust])-self.drag*abs(local)*local
            acceleration=self.R @ force+[0.,0.,9.81]+self.ba
            Av=-self.R @ D @ self.R.T
            At=-self.R @ (skew(force)+D @ skew(local))
        F[:3,3:6]=np.eye(3)*dt+.5*Av*dt*dt;F[:3,6:9]=.5*At*dt*dt
        F[3:6,3:6]+=Av*dt;F[3:6,6:9]=At*dt
        if motors is not None:
            F[:3,12:15]=np.eye(3)*(.5*dt*dt);F[3:6,12:15]=np.eye(3)*dt
        F[6:9,6:9]=rotation_step.T
        # Exact constant-rate gyro-bias derivative, evaluated once per small interval.
        phi=omega*dt;angle=np.linalg.norm(phi);A=skew(phi)
        Jr=np.eye(3)-.5*A+A@A/6 if angle<1e-5 else np.eye(3)-(1-np.cos(angle))/angle**2*A+(angle-np.sin(angle))/angle**3*A@A
        F[6:9,9:12]=-Jr*dt
        density=self.acceleration_noise**2 if motors is not None else 0.
        Q[:3,:3]=np.eye(3)*density*dt**3/3;Q[:3,3:6]=Q[3:6,:3]=np.eye(3)*density*dt**2/2
        Q[3:6,3:6]=np.eye(3)*density*dt
        Q[6:9,6:9]=np.eye(3)*self.gyro_noise**2*dt
        if angle_uncertainty is not None:Q[6:9,6:9]+=np.diag(np.asarray(angle_uncertainty)**2)
        Q[9:12,9:12]=np.eye(3)*self.gyro_bias_walk**2*dt
        Q[12:15,12:15]=np.eye(3)*self.force_bias_walk**2*dt
        self.p+=self.v*dt+.5*acceleration*dt*dt;self.v+=acceleration*dt;self.R=self.R @ rotation_step
        self.P=F @ self.P @ F.T+Q;self.P=.5*(self.P+self.P.T)
        return F

    def add_gate(self,key,relative_camera,right_camera):
        if key in self.gates:return
        world_camera=self.R @ self.C.T;center=self.p+world_camera @ relative_camera*(self.size/1.5)
        right=world_camera @ right_camera;yaw=float(np.arctan2(-right[0],right[1]))
        n=len(self.P);P=np.zeros((n+4,n+4));P[:n,:n]=self.P
        # PnP seeds the nonlinear mean only. A broad prior avoids counting the pixels twice.
        P[n:,n:]=np.diag([100.**2]*3+[np.pi**2]);self.P=P
        self.gates[key]=(n,center,yaw)

    def add_rim(self,width,height,fractional_spread):
        if self.rim is not None:return
        n=len(self.P);P=np.zeros((n+2,n+2));P[:n,:n]=self.P
        # P90-P10 spread is about2.563 standard deviations for a Gaussian.
        scale_derivative=np.array([width,height])/1.5;dimensions=scale_derivative*self.size
        sigma=dimensions*max(.01,fractional_spread/2.563)
        P[:n,n:]=self.P[:,15,None]*scale_derivative;P[n:,:n]=P[:n,n:].T
        P[n:,n:]=np.diag(sigma**2)+np.outer(scale_derivative,scale_derivative)*self.P[15,15]
        self.P=P;self.rim=(n,dimensions)

    def measurement(self,key,points,rim=False):
        if rim:
            width,height=self.rim[1];points=np.array([[0.,-width/2,-height],[0.,width/2,-height]])
        else:points=np.asarray(points)*(self.size/1.5)
        index,center,yaw=self.gates[key];gate=Rotation.from_euler('z',yaw).as_matrix()
        directions=np.asarray(points) @ gate.T;world=directions+center
        body=(world-self.p) @ self.R;camera=body @ self.C.T
        if np.min(camera[:,2])<=.1:return None
        h=camera @ self.K.T;pixels=h[:,:2]/h[:,2,None];H=np.zeros((2*len(points),len(self.P)))
        for i,(xyz,b,offset) in enumerate(zip(camera,body,directions)):
            x,y,z=xyz;J=np.array([[self.K[0,0]/z,0.,-self.K[0,0]*x/z**2],[0.,self.K[1,1]/z,-self.K[1,1]*y/z**2]])
            rows=slice(2*i,2*i+2);world_J=J @ self.C @ self.R.T
            H[rows,:3]=-world_J;H[rows,6:9]=J @ self.C @ skew(b)
            H[rows,index:index+3]=world_J;H[rows,index+3]=world_J @ np.cross([0.,0.,1.],offset)
            if rim:
                H[rows,self.rim[0]]=world_J @ gate @ [0.,-.5 if i==0 else .5,0.]
                H[rows,self.rim[0]+1]=world_J @ [0.,0.,-1.]
            else:H[rows,15]=world_J @ (offset/self.size)
        return pixels,H

    def observe(self,key,points,pixels,sigma_px=1.,noise=None,rim=False):
        prediction=self.measurement(key,points,rim)
        if prediction is None:return dict(accepted=False,reason='behind_camera')
        expected,H=prediction;residual=(np.asarray(pixels)-expected).ravel()
        V=np.eye(len(residual))*sigma_px**2 if noise is None else np.asarray(noise)
        S=H @ self.P @ H.T+V
        solved=np.linalg.solve(S,residual);nis=float(residual @ solved)
        # Chi-square 99.9% thresholds for four or eight pixel coordinates.
        threshold=18.467 if len(residual)==4 else 26.125
        result=dict(accepted=nis<=threshold,nis=nis,dof=len(residual),innovation_rms_px=float(np.sqrt(np.mean(residual**2))))
        if not result['accepted']:return result
        K=np.linalg.solve(S,H @ self.P).T;dx=K @ residual;A=np.eye(len(self.P))-K @ H
        covariance=A @ self.P @ A.T+K @ V @ K.T
        self.inject(dx);reset=np.eye(len(self.P));reset[6:9,6:9]-=.5*skew(dx[6:9])
        self.P=reset @ covariance @ reset.T;self.P=.5*(self.P+self.P.T)
        return result

    def relative(self,key):
        index,center,_=self.gates[key];relative=center-self.p;J=np.zeros((3,len(self.P)))
        J[:,:3]=-np.eye(3);J[:,index:index+3]=np.eye(3)
        camera=self.C @ self.R.T @ relative;D=self.C @ self.R.T @ J
        D[:,6:9]=self.C @ skew(self.R.T @ relative)
        return relative,J @ self.P @ J.T,camera,D @ self.P @ D.T


class GateHistory:
    """Rewind image updates and replay every subsequent process and pixel event."""
    def __init__(self,filter,stamp,gyro,seconds=1.):
        self.filter=filter;self.time=stamp;self.seconds=seconds
        self.gyros=[(stamp,np.asarray(gyro))];self.commands=[];self.observations=[]
        self.snapshots=[(stamp,filter.copy())];self.rejected_late=0;self.repropagated=0
        self.results={}

    def _rate(self,stamp):
        t=np.array([r[0] for r in self.gyros]);w=np.array([r[1] for r in self.gyros])
        return np.array([np.interp(stamp,t,w[:,i]) for i in range(3)])

    def _motor(self,stamp):
        index=np.searchsorted([r[0] for r in self.commands],stamp,side='right')-1
        return None if index<0 else self.commands[index][1]

    def _advance(self,stamp):
        if stamp>self.time:
            steps=max(1,int(np.ceil((stamp-self.time)/.02)))
            for end in np.linspace(self.time,stamp,steps+1)[1:]:
                middle=(self.time+end)/2
                first,last=self._rate(self.time),self._rate(end);dt=end-self.time
                self.filter.predict(dt,.5*(first+last),self._motor(middle),(last-first)*dt/np.sqrt(12));self.time=end

    def _replay(self,changed):
        choices=[i for i,(t,_) in enumerate(self.snapshots) if t<changed]
        if not choices:self.rejected_late+=1;return
        index=choices[-1];self.time,self.filter=self.snapshots[index][0],self.snapshots[index][1].copy()
        self.snapshots=self.snapshots[:index+1];end=self.gyros[-1][0]
        nodes=sorted({t for t,_ in self.gyros if self.time<t<=end}|{t for t,_ in self.commands if self.time<t<=end}|
            {o['time'] for o in self.observations if self.time<o['time']<=end})
        for stamp in nodes:
            self._advance(stamp)
            for row in self.observations:
                if row['time']==stamp:
                    if row['key'] not in self.filter.gates:
                        if row.get('seed') is None:continue
                        self.filter.add_gate(row['key'],*row['seed'])
                    if row.get('rim') and self.filter.rim is None:self.filter.add_rim(*row['rim_prior'])
                    self.results[row['id']]=self.filter.observe(row['key'],row['points'],row['pixels'],row.get('sigma_px',1.),rim=row.get('rim',False))
            self.snapshots.append((stamp,self.filter.copy()));self.repropagated+=1
        boundary=end-self.seconds;keep=max(0,np.searchsorted([t for t,_ in self.snapshots],boundary)-1)
        self.snapshots=self.snapshots[keep:];oldest=self.snapshots[0][0]
        self.observations=[o for o in self.observations if o['time']>oldest]
        i=max(0,np.searchsorted([r[0] for r in self.gyros],oldest)-1);self.gyros=self.gyros[i:]
        i=max(0,np.searchsorted([r[0] for r in self.commands],oldest)-1);self.commands=self.commands[i:]

    def imu(self,stamp,gyro):
        if stamp<=self.gyros[-1][0]:return
        self.gyros.append((stamp,np.asarray(gyro)));self._replay(stamp)

    def command(self,stamp,motors):
        self.commands.append((stamp,np.asarray(motors)));self.commands.sort(key=lambda r:r[0])
        if stamp<=self.time:self._replay(stamp)

    def observe(self,row):
        if row['time']<=self.snapshots[0][0]:self.rejected_late+=1;return
        self.observations.append(deepcopy(row));self.observations.sort(key=lambda o:(o['time'],o['id']))
        if row['time']<=self.time:self._replay(row['time'])
