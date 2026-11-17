"""Cold gate-map pose/velocity from pixels and short public IMU preintegration."""
import itertools
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp

POINTS=.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])


def locate(prior,views,imu,camera,intrinsics):
    """Compare all two-gate associations, without a launch pose or zero velocity.

    IMU rows: time, accelerationXYZ, gyroXYZ. Views contain time and measured
    quads sorted by area. Known-size map geometry gives scale; gravity in the
    IMU trajectory distinguishes otherwise similar repeated gate pairs.
    """
    import cv2
    if any(not {'time','pixels'}<=set(v)<={'time','pixels','image_id'} for v in views):raise ValueError('Unexpected localization observation fields')
    if len(views)<3 or views[-1]['time']-views[0]['time']<.5:return dict(accepted=False,reason='short_window')
    imu=np.asarray(imu,float);C=np.asarray(camera);K=np.asarray(intrinsics);stamp=views[-1]['time']
    if imu.ndim!=2 or imu.shape[1]!=7 or not np.isfinite(imu).all() or np.any(np.diff(imu[:,0])<=0):raise ValueError('Invalid public localization IMU window')
    if len(imu)<3 or imu[0,0]>views[0]['time'] or imu[-1,0]<stamp:return dict(accepted=False,reason='imu_window')
    matrices=[np.eye(3)]
    for first,last in zip(imu,imu[1:]):
        matrices.append(matrices[-1] @ Rotation.from_rotvec(.5*(first[4:]+last[4:])*(last[0]-first[0])).as_matrix())
    orient=Slerp(imu[:,0],Rotation.from_matrix(matrices));latest=orient(stamp).as_matrix()
    force=np.einsum('nij,nj->ni',np.array(matrices),imu[:,1:4]);dv=cumulative_trapezoid(force,imu[:,0],axis=0,initial=0.)
    dp=cumulative_trapezoid(dv,imu[:,0],axis=0,initial=0.);gravity=np.array([0.,0.,9.81]);times=np.array([v['time'] for v in views]);dt=times-imu[0,0]
    q=orient(times).as_matrix();pre=np.column_stack([np.interp(times,imu[:,0],dp[:,i]) for i in range(3)])
    world={g['id']:POINTS @ Rotation.from_euler('z',g['yaw']).as_matrix().T+g['center'] for g in prior['gates']}
    candidates=[];inverse=np.linalg.inv(K)
    for pair in itertools.combinations(range(len(views[-1]['pixels'])),2):
        if any(len(v['pixels'])<=max(pair) for v in views):continue
        pixels=np.array([np.concatenate([np.array(v['pixels'][j]) for j in pair]) for v in views])
        for ids in itertools.permutations(world,2):
            xyz=np.concatenate([world[k] for k in ids]);ok,r,t=cv2.solvePnP(xyz,pixels[-1],K,None,flags=cv2.SOLVEPNP_SQPNP)
            if not ok:continue
            seed=cv2.Rodrigues(r)[0].T @ C @ latest.T
            rays=np.c_[pixels.reshape(-1,2),np.ones(pixels.shape[0]*8)].reshape(-1,8,3) @ inverse.T
            directions=np.einsum('nij,nkj->nki',seed @ q @ C.T,rays);directions/=np.linalg.norm(directions,axis=2,keepdims=True)
            P=np.eye(3)-directions[:,:,:,None]*directions[:,:,None,:]
            offset=pre @ seed.T+.5*dt[:,None]**2*gravity
            design=np.concatenate((P,P*dt[:,None,None,None]),axis=3).reshape(-1,6);target=np.einsum('nkij,nkj->nki',P,xyz[None]-offset[:,None]).ravel()
            initial=np.linalg.lstsq(design,target,rcond=None)[0]
            def residual(value):
                R=seed @ Rotation.from_rotvec(value[6:9]).as_matrix();body=R @ q
                positions=value[:3]+dt[:,None]*value[3:6]+pre @ R.T+.5*dt[:,None]**2*gravity
                cam=np.einsum('nkj,nji,li->nkl',xyz[None]-positions[:,None],body,C)
                h=cam @ K.T
                return np.r_[(h[:,:,:2]/h[:,:,2,None]-pixels).ravel(),value[3:6]/10.]
            fit=least_squares(residual,np.r_[initial,np.zeros(3)],x_scale=[1.]*6+[.05]*3,max_nfev=40)
            R0=seed @ Rotation.from_rotvec(fit.x[6:9]).as_matrix();R=R0 @ latest;elapsed=stamp-imu[0,0]
            endpre=np.array([np.interp(stamp,imu[:,0],dp[:,i]) for i in range(3)]);endvel=np.array([np.interp(stamp,imu[:,0],dv[:,i]) for i in range(3)])
            position=fit.x[:3]+fit.x[3:6]*elapsed+R0 @ endpre+.5*gravity*elapsed**2;velocity=fit.x[3:6]+R0 @ endvel+gravity*elapsed
            if not fit.success or R[2,2]<.2 or min(((xyz-position) @ R @ C.T)[:,2])<.1:continue
            error=residual(fit.x);rms=float(np.sqrt(np.mean(error[:-3]**2)));verification=0.;extra_ids=[];extra_pairs=[];count=0
            reference=max(range(len(views)),key=lambda i:len(views[i]['pixels']));bodies=R0 @ q
            positions=fit.x[:3]+dt[:,None]*fit.x[3:6]+pre @ R0.T+.5*dt[:,None]**2*gravity
            remaining=set(world)-set(ids)
            for index,observed in enumerate(views[reference]['pixels']):
                if index in pair:continue
                projected=[]
                for key in remaining:
                    cam=(world[key]-positions[reference]) @ bodies[reference] @ C.T
                    if min(cam[:,2])<=.1:continue
                    h=cam @ K.T;difference=h[:,:2]/h[:,2,None]-observed
                    projected.append((float(np.sum(difference**2)),key))
                if not projected:verification+=1e4;continue
                _,key=min(projected);remaining.remove(key);extra_ids.append(key);extra_pairs.append(index)
                for j,v in enumerate(views):
                    if len(v['pixels'])<=index:continue
                    cam=(world[key]-positions[j]) @ bodies[j] @ C.T
                    if min(cam[:,2])<=.1:verification+=1e4;continue
                    h=cam @ K.T;verification+=float(np.sum((h[:,:2]/h[:,2,None]-v['pixels'][index])**2));count+=8
            current=[(i,k) for i,k in zip(extra_pairs,extra_ids) if i<len(views[-1]['pixels'])]
            assignment=[list(x) for x in sorted(zip(list(pair)+extra_pairs,list(ids)+extra_ids))]
            candidates.append(dict(score=float((error @ error+verification)/1.5**2),rms_px=rms,ids=list(ids),pair=list(pair),verification_ids=[k for i,k in current],verification_pairs=[i for i,k in current],
                verified_gates=len(extra_ids),expected_extra=len(views[reference]['pixels'])-len(pair),assignment=assignment,
                verification_rms_px=float(np.sqrt(verification/max(1,count))),position=position.tolist(),velocity=velocity.tolist(),rotation=R.tolist()))
    distinct={}
    for candidate in candidates:
        identity=tuple(tuple(x) for x in candidate['assignment'])
        if identity not in distinct or candidate['score']<distinct[identity]['score']:distinct[identity]=candidate
    candidates=sorted(distinct.values(),key=lambda c:c['score'])
    if not candidates:return dict(accepted=False,reason='no_association')
    best=candidates[0];margin=float(candidates[1]['score']-best['score']) if len(candidates)>1 else 1e12
    extra=best['expected_extra']
    confirmed=(best['verified_gates']==extra and best['verification_rms_px']<=2.5) if extra else stamp-views[0]['time']>=2.
    accepted=best['rms_px']<=1.5 and margin>13.816 and confirmed
    return dict(accepted=accepted,reason='matched' if accepted else 'ambiguous',time=stamp,span_s=stamp-views[0]['time'],margin=margin,hypotheses=candidates[:5],**best)


def bootstrap(config,imu,views,result=None):
    """Condition the latest pixels jointly with the uncertain map, once.

    Earlier images/IMU choose the nonlinear mean and association. Velocity keeps
    a broad3m/s covariance; no old flight pose or velocity enters this prior.
    """
    from .gate_filter import GateFilter,GateHistory
    C=np.array(config['camera_from_body']);K=np.array(config['intrinsics'])
    result=locate(config['map_prior'],views,imu,C,K) if result is None else result
    if not result['accepted']:raise ValueError('Cold localization is not resolved')
    f=GateFilter(result['rotation'],np.zeros(3),config['thrust_curve'],config['drag'],C,K)
    f.P[:3,:3]=np.eye(3)*1000.**2;f.P[3:6,3:6]=np.eye(3)*3.**2;f.P[6:9,6:9]=np.eye(3)*np.pi**2
    f.restore_map(config['map_prior']);f.p=np.array(result['position']);f.v=np.array(result['velocity'])
    f.localization=config.get('localization',False)
    stamp=result['time'];f.image_time_sigma=config.get('image_time_sigma_s',0.)
    rows=np.array(imu);f.angular_velocity=np.array([np.interp(stamp,rows[:,0],rows[:,i]) for i in range(4,7)]);f.begin_image()
    for key,index in zip(result['ids']+result.get('verification_ids',[]),result['pair']+result.get('verification_pairs',[])):
        accepted=f.observe(key,POINTS,np.array(views[-1]['pixels'][index]),sigma_px=1.5)
        if not accepted['accepted']:raise ValueError('Localized gate failed joint admission')
        f.last_visual[key]=stamp;f.last_full_visual[key]=stamp
    f.end_image();f.localization=config.get('localization',False);f.attitude_corrections=config['attitude_corrections'];f.broad_rim=config.get('broad_rim',False)
    f.partial_gate_births=config.get('partial_gate_births',True);f.upright_attitude=config.get('upright_attitude',False)
    gyro=np.array([np.interp(stamp,rows[:,0],rows[:,i]) for i in range(4,7)])
    h=GateHistory(f,stamp,gyro,seconds=config['history_s'])
    commands=np.array(config.get('bootstrap_commands',[]),float).reshape(-1,5)
    if not np.isfinite(commands).all() or np.any(np.diff(commands[:,0])<0) or np.any((commands[:,1:]<0)|(commands[:,1:]>1)):raise ValueError('Invalid bootstrap motor commands')
    h.commands=[(r[0],r[1:].copy()) for r in commands]
    for row in rows[rows[:,0]>stamp]:h.imu(row[0],row[4:7],row[1:4])
    return h,result
