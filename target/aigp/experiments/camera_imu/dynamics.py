"""Fit dynamics against camera-derived velocity change; no native labels."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear
from scipy.integrate import trapezoid
from scipy.spatial.transform import Rotation,Slerp
from target.aigp.experiments.camera_imu import vision

FIELDS={'time','camera_orientation','motors','image_time','image_gate','corners'}

def load(path):
    with np.load(path,allow_pickle=False) as f:
        if set(f.files)!=FIELDS:raise ValueError('Calibration input must contain exactly the public-data schema')
        data={k:f[k] for k in FIELDS}
    return data

def camera_velocities(data,p):
    times=data['time'];orient=Slerp(times,Rotation.from_matrix(data['camera_orientation']))
    samples=[]
    for stamp,gate,corners in zip(data['image_time'],data['image_gate'],data['corners']):
        side=float(np.min(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1)))
        if side<p['minimum_side_px'] or not times[0]<=stamp<=times[-1]:continue
        pose=vision.best_pose(corners)
        if pose is None or pose[0]>p['maximum_reprojection_px']:continue
        position=-orient(stamp).apply(pose[2]);samples.append((stamp,gate,position,side))
    velocities=[];half=p['velocity_fit_window_s']/2
    for t,gate,_,side in samples:
        nearby=[s for s in samples if s[1]==gate and abs(s[0]-t)<=half]
        if len(nearby)<p['minimum_fit_images'] or nearby[-1][0]-nearby[0][0]<.3:continue
        stamp=np.array([s[0] for s in nearby]);x=stamp-t
        if np.max(np.diff(stamp))>p['maximum_image_gap_s']:continue
        design=np.c_[np.ones(len(x)),x,x*x];position=np.array([s[2] for s in nearby]);weights=np.ones(len(x))
        for _ in range(4):
            square=np.sqrt(weights);fit=np.linalg.lstsq(design*square[:,None],position*square[:,None],rcond=None)[0]
            residual=np.linalg.norm(design @ fit-position,axis=1);weights=np.minimum(1.,.03/np.maximum(residual,1e-9))
        if np.linalg.norm(fit[1])>8.:continue
        velocities.append((t,gate,fit[1],side))
    return velocities

def windows(data,p,lag):
    velocities=camera_velocities(data,p);orient=Slerp(data['time'],Rotation.from_matrix(data['camera_orientation']));result=[]
    for gate in sorted(set(v[1] for v in velocities)):
        group=[v for v in velocities if v[1]==gate];ts=np.array([v[0] for v in group]);vs=np.array([v[2] for v in group]);last=-np.inf
        for i,t0 in enumerate(ts):
            if t0-last<p['pair_stride_s']:continue
            j=np.searchsorted(ts,t0+p['velocity_change_window_s'])
            if j>=len(ts):continue
            t1=ts[j]
            if t1-t0>.6 or np.max(np.diff(ts[i:j+1]))>p['maximum_image_gap_s']:continue
            grid=np.linspace(t0,t1,max(3,int(np.ceil((t1-t0)/.01))+1));body=orient(grid).as_matrix() @ vision.CAMERA_FROM_BODY
            indices=np.searchsorted(data['time'],grid-lag,side='right')-1
            if indices.min()<0:continue
            motors=data['motors'][indices];velocity=np.array([np.interp(grid,ts,vs[:,k]) for k in range(3)]).T
            local=np.einsum('nji,nj->ni',body,velocity);drag=abs(local)*local
            thrust=-body[:,:,2]
            xy=-np.einsum('nij,nj->ni',body,np.c_[drag[:,:2],np.zeros(len(grid))])
            z=-body[:,:,2]*drag[:,2,None]
            features=np.stack([thrust*motors.mean(axis=1)[:,None],thrust*np.mean(motors**2,axis=1)[:,None],xy,z],axis=2)
            design=trapezoid(features,grid,axis=0);target=vs[j]-vs[i]-np.array([0.,0.,9.81])*(t1-t0)
            result.append(dict(time=float(t0),gate=int(gate),dt=float(t1-t0),design=design,target=target,delta_velocity=vs[j]-vs[i],mean_motor=float(motors.mean()),mean_side=float(np.median([x[3] for x in group[i:j+1]]))))
            last=t0
    return result

def fit(groups):
    rows=[row for group in groups for row in group];design=np.concatenate([r['design'] for r in rows]);target=np.concatenate([r['target'] for r in rows]);scale=np.linalg.norm(design,axis=0);weights=np.ones(len(rows))
    for _ in range(8):
        w=np.repeat(np.sqrt(weights),3);solution=lsq_linear(design*w[:,None]/scale,target*w,bounds=(0.,np.inf),tol=1e-11).x/scale
        errors=np.array([np.linalg.norm(r['design'] @ solution-r['target']) for r in rows]);weights=np.minimum(1.,.2/np.maximum(errors,1e-9))
    return solution,float(np.linalg.cond(design/scale))

def score(rows,theta):
    error=np.array([r['design'] @ theta-r['target'] for r in rows]);cv=np.array([r['delta_velocity'] for r in rows]);norm=np.linalg.norm(error,axis=1)
    return dict(windows=len(rows),rms_m_s=float(np.sqrt(np.mean(norm**2))),median_m_s=float(np.median(norm)),p95_m_s=float(np.quantile(norm,.95)),constant_velocity_rms_m_s=float(np.sqrt(np.mean(np.sum(cv**2,axis=1)))),axis_rms_m_s=np.sqrt(np.mean(error**2,axis=0)).tolist())

def run(root):
    manifest=json.loads((root/'manifest.json').read_text());p=manifest['protocol'];cache={}
    for item in manifest['training']+manifest['heldout']:
        path=root/item['path'];assert hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256'];cache[item['path']]=load(path)
    alternatives=[]
    for lag in p['lag_candidates_s']:
        train=[windows(cache[x['path']],p,lag) for x in manifest['training']];fold=[]
        for i in range(len(train)):
            theta,_=fit([g for j,g in enumerate(train) if j!=i]);fold.append(score(train[i],theta)['rms_m_s'])
        theta,condition=fit(train);alternatives.append(dict(lag_s=lag,training_cv_rms_m_s=float(np.mean(fold)),fold_rms_m_s=fold,theta=theta.tolist(),condition=condition,training=[score(g,theta) for g in train]))
    best=min(alternatives,key=lambda a:a['training_cv_rms_m_s']);theta=np.array(best['theta']);lag=best['lag_s']
    # Freeze the selected parameters before evaluating any heldout camera labels.
    result=dict(selected=best,alternatives=alternatives,input_manifest_sha256=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),label_source='Camera corner pose differences with public gyro/visual attitude; no fused p/v or native fields')
    (root/'training-fit.json').write_text(json.dumps(result,indent=2)+'\n')
    result['heldout']=[dict(source=x['source'],**score(windows(cache[x['path']],p,lag),theta)) for x in manifest['heldout']]
    (root/'fit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


def shared_drag(root,output):
    """Reduce weakly identified drag freedom using training-flight CV only."""
    manifest_path=root/'manifest.json';manifest=json.loads(manifest_path.read_text())
    separate=json.loads((root/'training-fit.json').read_text())
    manifest_hash=hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    if manifest_hash!=separate['input_manifest_sha256']:raise ValueError('Training manifest changed')
    p=manifest['protocol'];lag=separate['selected']['lag_s']
    def inputs(item):
        path=root/item['path']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Input hash changed')
        return windows(load(path),p,lag)
    raw=[inputs(item) for item in manifest['training']]
    train=[[dict(r,design=np.column_stack((r['design'][:,:2],r['design'][:,2]+r['design'][:,3]))) for r in group] for group in raw]
    folds=[];parameters=[]
    for i in range(len(train)):
        theta,_=fit([g for j,g in enumerate(train) if j!=i])
        folds.append(score(train[i],theta)['rms_m_s']);parameters.append(theta.tolist())
    theta,condition=fit(train);cv=float(np.mean(folds));eligible=cv<=1.05*separate['selected']['training_cv_rms_m_s']
    rule='Prefer shared drag if leave-flight-out training RMS is within5% of separate-drag RMS; keep its selected lag.'
    frozen=dict(rule=rule,input_manifest_sha256=manifest_hash,lag_s=lag,shared_parameters=theta.tolist(),
        training_cv_rms_m_s=cv,separate_drag_training_cv_rms_m_s=separate['selected']['training_cv_rms_m_s'],
        fold_rms_m_s=folds,fold_parameters=parameters,condition=condition,eligible=eligible)
    training_path=root/'shared-drag-training.json';training_path.write_text(json.dumps(frozen,indent=2)+'\n')
    if not eligible:raise ValueError('Shared drag failed the fixed training selection rule')
    a,b,drag=theta;theta4=np.array([a,b,drag,drag])
    # Parameters are frozen on disk before reading any heldout labels.
    heldout=[dict(source=item['source'],**score(inputs(item),theta4)) for item in manifest['heldout']]
    hover=2*9.81/(a+np.sqrt(a*a+4*b*9.81));support=[r['mean_motor'] for group in raw for r in group]
    profile=dict(schema=1,label='public-camera-shared-drag',label_source=separate['label_source'],lag_s=lag,
        source_fit_sha256=hashlib.sha256(training_path.read_bytes()).hexdigest(),manifest_sha256=manifest_hash,
        position_config=dict(hover_thrust=float(hover),thrust_acceleration=float(a+2*b*hover),position_gain=4/3,
            velocity_gain=3.,attitude_gain=8.,max_rate=3.,thrust_curve=[float(a),float(b)],body_resistance_per_m=[float(drag)]*3),
        training_inputs=manifest['training'],heldout=heldout,selection=rule,
        limits=dict(training_windows=sum(map(len,raw)),heldout_windows=sum(x['windows'] for x in heldout),
            fold_parameters=parameters,parameter_units=['m/s2 per u','m/s2 per u2','1/m'],
            operating_mean_motor_range=[min(support),max(support)],nonnegative_scaled_design_condition=condition),
        provenance=manifest['provenance'],heldout_status='These heldout flights were previously inspected during model development; fresh native flights are the prospective test.')
    output.write_text(json.dumps(profile,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),training_cv_rms_m_s=cv,parameters=theta4.tolist(),heldout=heldout),indent=2))
    return profile

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('--shared-drag',type=Path,metavar='MODEL')
    args=parser.parse_args()
    if args.shared_drag:shared_drag(args.root,args.shared_drag)
    else:run(args.root)
