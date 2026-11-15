"""Native references enter only here, after joint replay outputs are frozen."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def statistics(x):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    return dict(count=len(x),rms=float(np.sqrt(np.mean(x*x))),median=float(np.median(x)),p95=float(np.quantile(abs(x),.95)),maximum=float(max(abs(x)))) if len(x) else dict(count=0)


def score(source,root,render=True):
    estimate_hash=hashlib.sha256((root/'estimates.npz').read_bytes()).hexdigest()
    with gzip.open(root/'estimates.jsonl.gz','rt') as stream:joint=list(map(json.loads,stream))
    reference=source/'reference' if (source/'reference').exists() else source
    with gzip.open(reference/'control.jsonl.gz','rt') as stream:baseline=list(map(json.loads,stream))
    native_path=reference/'state-errors.csv'
    if not native_path.exists():native_path=reference/'state-errors.csv.gz'
    native=np.genfromtxt(native_path,delimiter=',',names=True);native=native[np.r_[True,np.diff(native['time'])>0]]
    joint=[r for r in joint if native['time'][0]<=r['time']<=native['time'][-1] and 'camera_gate' in r]
    t=np.array([r['time'] for r in joint]);pos=np.array([r['position'] for r in joint]);vel=np.array([r['velocity'] for r in joint]);rot=np.array([r['rotation'] for r in joint])
    cov=np.array([r['state_covariance'] for r in joint]);key=np.array([r['gate'] for r in joint]);nt=native['time']
    npos=np.array([np.interp(t,nt,native['native_position_'+a]) for a in 'ned']).T
    nvel=np.array([np.interp(t,nt,native['native_velocity_'+a]) for a in 'ned']).T
    nrot=Slerp(nt,Rotation.from_euler('xyz',np.column_stack([native['native_attitude_'+a+'_deg'] for a in ['roll','pitch','yaw']]),degrees=True))(t).as_matrix()
    # Recover the baseline scorer's fixed initial heading gauge. Do not fit away this filter's yaw error.
    from .vision import CAMERA_FROM_BODY
    first=next(r for r in baseline if 'camera_orientation' in r and abs(r['imu_time']-nt[0])<1e-9)
    scored_body=Rotation.from_euler('xyz',[native['estimated_attitude_'+a+'_deg'][0] for a in ['roll','pitch','yaw']],degrees=True).as_matrix()
    local_body=np.array(first['camera_orientation']) @ CAMERA_FROM_BODY
    yaw=Rotation.from_matrix(scored_body @ local_body.T).as_euler('xyz')[2]
    align=Rotation.from_euler('z',yaw).as_matrix();origin=npos[0]-align @ pos[0]
    p=pos @ align.T+origin;v=vel @ align.T;R=align @ rot
    perr=p-npos;verr=v-nvel;aerr=Rotation.from_matrix(np.swapaxes(R,1,2) @ nrot).as_rotvec()
    from .geometry import gate_openings
    gates=gate_openings()[:6].mean(axis=1);truth_relative=gates[key-1]-npos
    truth_camera=np.einsum('ij,nkj,nk->ni',CAMERA_FROM_BODY,nrot,truth_relative)
    depth=np.array([r['camera_gate'][2] for r in joint]);derr=depth-truth_camera[:,2]
    Pp=align @ cov[:,:3,:3] @ align.T;Pv=align @ cov[:,3:6,3:6] @ align.T
    sigma=dict(position=np.sqrt(np.diagonal(Pp,axis1=1,axis2=2)),velocity=np.sqrt(np.diagonal(Pv,axis1=1,axis2=2)),
        attitude=np.sqrt(np.diagonal(cov[:,6:9,6:9],axis1=1,axis2=2)),depth=np.sqrt([r['camera_gate_covariance'][2][2] for r in joint]))
    bt=np.array([r['imu_time'] for r in baseline]);bgate=np.array([r.get('gate_number',1) for r in baseline])
    braking=[r['imu_time'] for r in baseline if r.get('brake_test_active')]
    if not braking:braking=[r['imu_time'] for r in baseline if r.get('gate_number')==4 and r.get('crossing_alignment')]
    lo,hi=(min(braking),max(braking)) if braking else (t[-1],t[-1])
    masks=dict(full=np.ones(len(t),bool),gate4=key==4,braking=(t>=lo)&(t<=hi))
    errors=dict(position=np.linalg.norm(perr,axis=1),velocity=np.linalg.norm(verr,axis=1),attitude=np.degrees(np.linalg.norm(aerr,axis=1)),depth=derr)
    berrors={k:np.interp(t,nt,native[field]) for k,field in [('position','position_error_m'),('velocity','velocity_error_m_s'),('attitude','attitude_error_deg')]}
    depth_rows=[r for r in baseline if 'gate_center' in r and 'camera_orientation' in r]
    bdtime=np.array([r['imu_time'] for r in depth_rows]);bdepth=np.array([(np.array(r['camera_orientation']).T @ (np.array(r['gate_center'])-r['position']))[2] for r in depth_rows])
    berrors['depth']=np.interp(t,bdtime,bdepth)-truth_camera[:,2]
    components=dict(position=perr,velocity=verr,attitude=aerr,depth=derr)
    metrics={}
    for name,mask in masks.items():
        metrics[name]=dict(samples=int(mask.sum()),joint={k:statistics(val[mask]) for k,val in errors.items()},baseline={k:statistics(val[mask]) for k,val in berrors.items()},
            joint_3sigma_coordinate_coverage={k:float(np.mean(abs(val[mask])<=3*sigma[k][mask])) if mask.any() else None for k,val in components.items()})
    metrics.update(braking_window_imu_s=[lo,hi],alignment=dict(yaw_rad=float(yaw),origin_m=origin.tolist(),scale=1.,note='Same fixed initial heading gauge as the recorded baseline; origin at first shared scorer epoch. No fit to tested attitude.'),
        estimate_sha256_before_scoring=estimate_hash,native_used_for='Scoring only; no outputs returned to replay',
        comparison='Same recorded commands and camera candidates; this replay cannot establish closed-loop flight performance.')
    (root/'score.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    if not render:return metrics
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(4,2,figsize=(14,12),layout='constrained')
    for i,k in enumerate(['position','velocity','attitude','depth']):
        s=sigma[k];bound=3*s if s.ndim==1 else 3*np.linalg.norm(s,axis=1)
        if k=='attitude':bound=np.degrees(bound)
        for j,mask in enumerate([masks['full'],masks['braking']]):
            a=ax[i,j];a.plot(t[mask],abs(berrors[k][mask]),label='existing estimator');a.plot(t[mask],abs(errors[k][mask]),label='joint filter')
            a.plot(t[mask],bound[mask],':',color='C2',label='3σ [norm bound for vectors]');a.set_ylabel(k+' error '+dict(position='[m]',velocity='[m/s]',attitude='[deg]',depth='[m]')[k]);a.grid(alpha=.2);a.legend(fontsize=7)
            a.set_xlabel('public IMU epoch [s]')
            if j==0:a.axvspan(lo,hi,color='k',alpha=.05)
    ax[0,0].set_title('Full scored flight; fixed origin/yaw');ax[0,1].set_title('Braking, defined by recorded public controller phase')
    fig.suptitle('Joint position / velocity / attitude / gate depth | frozen braking replay')
    fig.savefig(root/'errors.png',dpi=130);plt.close(fig)
    np.savez_compressed(root/'scored-errors.npz',time=t,position_error=perr,velocity_error=verr,attitude_error=aerr,depth_error=derr,
        gate=key,position_sigma=sigma['position'],velocity_sigma=sigma['velocity'],attitude_sigma=sigma['attitude'],depth_sigma=sigma['depth'])
    print(json.dumps({k:metrics[k] for k in ['full','braking']},indent=2));return metrics


def render_live(root,n,rows,post,true_depth,depth,depth_sigma,braking):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .vision import CAMERA_FROM_BODY
    t=n['time']-n['time'][0]
    estR=Rotation.from_euler('xyz',[n['estimated_attitude_'+a+'_deg'][0] for a in ['roll','pitch','yaw']],degrees=True).as_matrix()
    direction=estR @ (np.array(rows[0]['camera_orientation']) @ CAMERA_FROM_BODY).T @ [1.,0.,0.]
    np0=np.array([n['native_position_'+a][0] for a in 'ned'])
    native_p=np.column_stack([n['native_position_'+a] for a in 'ned']);estimated_p=np.column_stack([n['estimated_position_'+a] for a in 'ned'])
    native_v=np.column_stack([n['native_velocity_'+a] for a in 'ned']);estimated_v=np.column_stack([n['estimated_velocity_'+a] for a in 'ned'])
    values=[((estimated_p-np0) @ direction,(native_p-np0) @ direction,'Forward position [m]'),
        (estimated_v @ direction,native_v @ direction,'Forward velocity [m/s]'),
        (n['estimated_attitude_pitch_deg'],n['native_attitude_pitch_deg'],'Body pitch [deg]'),(depth,true_depth,'Gate camera depth [m]')]
    fig,ax=plt.subplots(4,2,figsize=(13,11),layout='constrained');fig.suptitle(root.parent.name+'/'+root.name+' | joint feedback; shaded interval is braking')
    for i,(estimate,truth,label) in enumerate(values):
        ax[i,0].plot(t,truth,'--',label='native scoring reference');ax[i,0].plot(t,estimate,label='joint estimate');ax[i,0].set_ylabel(label)
        if i<3:
            key=['position','velocity','attitude'][i];error=n[key+'_error_'+('deg' if i==2 else 'm_s' if i==1 else 'm')]
            bound=3*np.sqrt([np.trace(p[key+'_covariance']) for p in post])
            if i==2:bound=np.degrees(bound)
        else:error=abs(depth-true_depth);bound=3*depth_sigma
        ax[i,1].plot(t,error,label='scored error');ax[i,1].plot(t,bound,':',label='3 sqrt(trace covariance)' if i<3 else '3 sigma depth')
        ax[i,1].set_ylabel(['Position error norm [m]','Velocity error norm [m/s]','Attitude error norm [deg]','Absolute depth error [m]'][i])
    for a in ax.flat:
        if braking.any():a.axvspan(min(t[braking]),max(t[braking]),color='k',alpha=.08)
        a.set_xlabel('Elapsed since approach [s]');a.legend(fontsize=7);a.grid(alpha=.2)
    output=root/'plots';output.mkdir(exist_ok=True);fig.savefig(output/'joint.png',dpi=130);plt.close(fig)


def score_live(root):
    from .geometry import gate_openings
    from .vision import CAMERA_FROM_BODY
    if not (root/'state-errors.csv').exists():
        result=dict(scoring_status='No approach reference; retain startup failure',brake_resumed=False)
        (root/'joint-score.json').write_text(json.dumps(result)+'\n');return result
    with gzip.open(root/'control.jsonl.gz','rt') as f:rows=[r for r in map(json.loads,f) if r['phase'] in ['TRACKING','FINAL_IMU','HANDOFF'] and r.get('joint_posterior')]
    n=np.genfromtxt(root/'state-errors.csv',delimiter=',',names=True)
    assert len(rows)==len(n);np.testing.assert_array_equal([r['imu_time'] for r in rows],n['time'])
    t=n['time'];post=[r['joint_posterior'] for r in rows];gate=gate_openings().mean(axis=1)[np.array([r['gate_number'] for r in rows])-1]
    nr=Rotation.from_euler('xyz',np.column_stack([n['native_attitude_'+a+'_deg'] for a in ['roll','pitch','yaw']]),degrees=True).as_matrix()
    pos=np.column_stack([n['native_position_'+a] for a in 'ned']);vel=np.column_stack([n['native_velocity_'+a] for a in 'ned'])
    truth=np.einsum('ij,nkj,nk->ni',CAMERA_FROM_BODY,nr,gate-pos);depth=np.array([p['camera_gate'][2] for p in post]);ds=np.sqrt([p['camera_covariance'][2][2] for p in post]);de=depth-truth[:,2]
    braking=np.array([bool(r.get('brake_test_active')) for r in rows]);resumed=np.array([r.get('brake_test_state')=='resumed' for r in rows])
    errors=dict(position=n['position_error_m'],velocity=n['velocity_error_m_s'],attitude=n['attitude_error_deg'],depth=de)
    result={name:dict(samples=int(mask.sum()),errors={key:statistics(value[mask]) for key,value in errors.items()},depth_3sigma_coverage=float(np.mean(abs(de[mask])<=3*ds[mask])) if mask.any() else None) for name,mask in [('full',np.ones(len(t),bool)),('braking',braking),('after_braking',resumed)]}
    if braking.any():result['braking_interval_s']=[float(min(t[braking])),float(max(t[braking]))]
    result['brake_resumed']=bool(resumed.any());result['reference_use']='Native scoring only; commanded flight used the joint public-input posterior.'
    (root/'joint-score.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(root/'joint-score-arrays.npz',time=t,depth=depth,true_depth=truth[:,2],depth_sigma=ds,depth_error=de,position_error=errors['position'],velocity_error=errors['velocity'],attitude_error=errors['attitude'],braking=braking,
            down_velocity=np.array([r['velocity'][2] for r in rows]),true_down_velocity=vel[:,2],down_velocity_sigma=np.sqrt([p['velocity_covariance'][2][2] for p in post]))
    render_live(root,n,rows,post,truth[:,2],depth,ds,braking)
    print(root.name,json.dumps(result['braking']));return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path,nargs='?');parser.add_argument('--live',action='store_true');args=parser.parse_args()
    if args.live:score_live(args.source)
    elif args.output is None:parser.error('replay scoring requires an output directory')
    else:score(args.source,args.output)
