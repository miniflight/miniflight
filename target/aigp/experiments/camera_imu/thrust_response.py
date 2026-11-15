"""Independent known-size camera displacement response, public gyro and own inputs."""
import gzip,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import lsq_linear
from miniflight.gate_filter import upright_seed
from target.aigp.experiments.camera_imu.vision import CAMERA_FROM_BODY as C,K
PUBLIC={'imu_time','imu_received_at','acceleration','gyro','frame_id','frame_header_time_ns','frame_time_estimated_ns',
 'frame_received_at','frame_shape_hw','motor_sent_host_time','motor_outputs'}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stats(x):
    return dict(count=len(x),mae=float(np.mean(abs(x))),rms=float(np.sqrt(np.mean(x*x))),p95=float(np.quantile(abs(x),.95)),maximum=float(max(abs(x)))) if len(x) else dict(count=0)
def command_times(sent,received,stamp):
    result=[]
    for host in sent:
        k=np.searchsorted(received,host,side='right')
        if not k:raise ValueError('Command precedes the public clock')
        result.append(host-min(received[max(0,k-256):k]-stamp[max(0,k-256):k]))
    return np.array(result)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RULE='Six command levels, no parameter refit: future camera-position RMS<=0.12m, >=15% reduction from prior, each pulse RMS<=0.20m.'

def conditional_system(X,y,B,pre,post):
    initial=np.linalg.pinv(B[pre])
    return X[post]-B[post] @ initial @ X[pre],y[post]-B[post] @ initial @ y[pre]

def fit_parameters(X,y):
    scale=np.linalg.norm(X,axis=0)
    return lsq_linear(X/scale,y,bounds=(0,np.inf),tol=1e-12).x/scale

def validate_provenance(meta,model,prior_hash):
    if meta['public_inputs_sha256']==model['training_inputs']['public_inputs_sha256']:raise ValueError('Validation must use a separate flight')
    if prior_hash!=model['baseline_sha256']:raise ValueError('Baseline changed after freezing the model')

def windows(source,prior,accepted_only=True):
    with np.load(source/'public-inputs.npz',allow_pickle=False) as f:
        if set(f.files)!=PUBLIC:raise ValueError('Unexpected public-input schema')
        d={k:f[k] for k in f.files}
    with gzip.open(source/'control.jsonl.gz','rt') as h:rows=[json.loads(l) for l in h]
    rows=[r for r in rows if 'motor_commands' in r]
    assert np.array_equal(d['motor_outputs'],[r['motor_commands'] for r in rows])
    ts=d['imu_time'];ct=command_times(d['motor_sent_host_time'],d['imu_received_at'],ts)
    seed=(ts>=ct[0]-.4)&(ts<ct[0]-.02);gravity=-d['acceleration'][seed].mean(axis=0);gravity/=np.linalg.norm(gravity)
    cross=np.cross(gravity,[0.,0.,1.]);R=Rotation.from_rotvec(cross*np.arctan2(np.linalg.norm(cross),gravity[2])/max(np.linalg.norm(cross),1e-12)).as_matrix()
    rotations=[R.copy()]
    # Integrate forward and backward from the stationary pre-command epoch.
    i0=np.where(seed)[0][-1];rotations=[None]*len(ts);rotations[i0]=R.copy()
    for i in range(i0+1,len(ts)):
        R=R @ Rotation.from_rotvec(.5*(d['gyro'][i-1]+d['gyro'][i])*(ts[i]-ts[i-1])).as_matrix();rotations[i]=R.copy()
    R=rotations[i0]
    for i in range(i0-1,-1,-1):
        R=R @ Rotation.from_rotvec(-.5*(d['gyro'][i+1]+d['gyro'][i])*(ts[i+1]-ts[i])).as_matrix();rotations[i]=R.copy()
    orient=Slerp(ts,Rotation.from_matrix(np.array(rotations)));seen=set();images=[]
    selected={r['image_id'] for r in rows if r.get('image_time') is not None and r.get('full_image_time')==r['image_time']}
    with gzip.open(source/'public-observations.jsonl.gz','rt') as h:observations=[json.loads(l) for l in h]
    for o in observations:
        if o['kind']!='aperture' or (accepted_only and o['image_id'] not in selected) or o['image_id'] in seen:continue
        seen.add(o['image_id']);t=o['capture_time'];corners=np.array(o['points']);side=min(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1))
        if not ts[0]<=t<=ts[-1] or side<30 or o['quality']['reprojection_px']>.5:continue
        R=orient(t).as_matrix();seed_pose=upright_seed(corners,C @ R.T @ [0.,0.,1.],K)
        if seed_pose is None:continue
        images.append((t,-R @ C.T @ seed_pose[0]))
    it=np.array([r[0] for r in images]);p=np.array([r[1] for r in images]);u=d['motor_outputs'];moments=np.c_[u.mean(axis=1),(u*u).mean(axis=1)]
    ex=[r.get('excitation',{}) for r in rows];groups=sorted({e['index'] for e in ex if e.get('applied')});result=[]
    for key in groups:
        own=np.array([i for i,e in enumerate(ex) if e.get('applied') and e['index']==key]);start=ct[own[0]];end=ct[own[-1]+1]
        # Rate hold lasts400ms after the pulse. End before attitude feedback resumes.
        mask=(it>=start-.4)&(it<=end+.35)
        if sum(mask)<15 or sum(mask&(it<start))<5:continue
        times=it[mask];position=p[mask,2];lo=times[0];grid=np.arange(lo,times[-1]+.0025,.0025)
        R=orient(grid).as_matrix();mi=np.searchsorted(ct,grid,side='right')-1
        basis=-R[:,2,2,None]*moments[mi]
        # Freeze the drag linearization using pre-pulse images and the prior model.
        # Future camera positions never enter this predicted correction.
        config=json.loads(prior.read_text())['position_config'];before=times<start-.025
        v0=np.polyfit(times[before]-lo,position[before],1)[0];vz=np.zeros(len(grid));vz[0]=v0
        for j in range(1,len(grid)):
            v=vz[j-1];az=9.81+basis[j-1] @ config['thrust_curve']-config['body_resistance_per_m'][2]*abs(v)*v
            vz[j]=v+az*(grid[j]-grid[j-1])
        drag=config['body_resistance_per_m'][2]*abs(vz)*vz
        first=cumulative_trapezoid(basis,grid,axis=0,initial=0.);double=cumulative_trapezoid(first,grid,axis=0,initial=0.)
        dg=cumulative_trapezoid(cumulative_trapezoid(drag,grid,initial=0.),grid,initial=0.)
        t=times-lo;X=np.column_stack([np.interp(times,grid,double[:,i]) for i in range(2)])
        y=position-.5*9.81*t*t+np.interp(times,grid,dg);B=np.c_[np.ones(len(t)),t]
        pre=times<start-.025;post=times>=start
        design,target=conditional_system(X,y,B,pre,post)
        result.append(dict(pulse=key,start=start,time=times,position=position,X=X,y=y,B=B,
            design=design,target=target,level=ex[own[0]]['collective'],pre=pre,post=post))
    return result,dict(public_inputs_sha256=digest(source/'public-inputs.npz'),public_controls_sha256=digest(source/'control.jsonl.gz'),
        public_observations_sha256=digest(source/'public-observations.jsonl.gz'),images=len(images),windows=len(result))

def run(source,output,prior,frozen=None):
    output.mkdir(parents=True,exist_ok=True);w,meta=windows(source,prior,accepted_only=frozen is None)
    complete=len(w)>=6 and len({r['level'] for r in w})>=6
    if not w or (frozen is None and not complete):raise ValueError('Need six public pulse windows covering all declared levels')
    X=np.concatenate([r['design'] for r in w]);y=np.concatenate([r['target'] for r in w]);scale=np.linalg.norm(X,axis=0)
    if frozen is not None:validate_provenance(meta,json.loads(frozen.read_text()),digest(prior))
    theta=fit_parameters(X,y) if frozen is None else np.array(json.loads(frozen.read_text())['position_config']['thrust_curve'])
    baseline=np.array(json.loads(prior.read_text())['position_config']['thrust_curve']);folds=[]
    for i,r in enumerate(w) if frozen is None else []:
        x=np.concatenate([q['design'] for j,q in enumerate(w) if j!=i]);z=np.concatenate([q['target'] for j,q in enumerate(w) if j!=i]);s=np.linalg.norm(x,axis=0)
        th=fit_parameters(x,z)
        folds.append(dict(pulse=r['pulse'],theta=th.tolist(),candidate=stats(r['design'] @ th-r['target']),baseline=stats(r['design'] @ baseline-r['target'])))
    report=dict(inputs=meta,theta=theta.tolist(),condition=float(np.linalg.cond(X/scale)),candidate=stats(X @ theta-y),baseline=stats(X @ baseline-y),folds=folds,
        metric='Future camera-derived vertical position error. Initial position/velocity use only images before each pulse; no future state fit.',
        prior_sha256=digest(prior),training=frozen is None,complete=complete,
        image_selection='Accepted training associations' if frozen is None else 'Raw measured aperture corners with geometric quality checks; no estimator acceptance filter',
        windows=[dict(pulse=r['pulse'],level=r['level'],candidate=stats(r['design'] @ theta-r['target']),baseline=stats(r['design'] @ baseline-r['target'])) for r in w])
    if frozen is None:
        prior_data=json.loads(prior.read_text());hover=float(2*9.81/(theta[0]+np.sqrt(theta[0]**2+4*theta[1]*9.81)))
        model=dict(schema=2,label='public-camera-thrust-response',label_source='Known-size measured gate pixels, public gyro, own sent commands. Raw acceleration only supplies stationary initial gravity; no native labels or fused p/v.',
            position_config=prior_data['position_config'],lag_s=0.,training_inputs=meta,baseline_sha256=digest(prior),fitter_sha256=digest(Path(__file__)),
            supported_mean_command=[min(r['level'] for r in w),max(r['level'] for r in w)],
            validation_rule=RULE,frozen_assumptions='Camera mount, image clock mapping, drag and instantaneous command model held fixed. Drag linearization uses only pre-pulse images and prior dynamics. Pulse grouping is the validation unit.')
        model['position_config'].update(thrust_curve=theta.tolist(),hover_thrust=hover,thrust_acceleration=float(theta[0]+2*theta[1]*hover))
        (output/'model.json').write_text(json.dumps(model,indent=2)+'\n')
    else:
        report.update(model_sha256=digest(frozen),refit=False)
        report['accepted']=bool(complete and report['candidate']['rms']<=.12 and report['candidate']['rms']<=.85*report['baseline']['rms'] and max(r['candidate']['rms'] for r in report['windows'])<=.2)
        report['acceptance_rule']=RULE
    (output/'camera-fit.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,ax=plt.subplots(2,3,figsize=(14,8),layout='constrained')
    for a,r in zip(ax.flat,w):
        for th,label in [(baseline,'old'),(theta,'camera fit')]:
            pre=r['pre'];state=np.linalg.lstsq(r['B'][pre],r['y'][pre]-r['X'][pre] @ th,rcond=None)[0];prediction=r['X'] @ th+r['B'] @ state-r['y']+r['position'];a.plot(r['time']-r['start'],prediction,label=label)
        a.plot(r['time']-r['start'],r['position'],'.',label='known-size pixels + gyro');a.set_title(f"pulse{r['pulse']}, command{r['level']}");a.set(xlabel='Since pulse[s]',ylabel='Gate-relative down position[m]');a.grid(alpha=.2);a.legend(fontsize=7)
    fig.suptitle('Frozen public camera response prediction | initial state from pre-pulse images only');fig.savefig(output/'camera-response.png',dpi=130);plt.close(fig)
    print(json.dumps(dict(parameters=theta.tolist(),candidate=report['candidate'],baseline=report['baseline'],accepted=report.get('accepted')),indent=2));return report

if __name__=='__main__':
    import argparse
    a=argparse.ArgumentParser();a.add_argument('source',type=Path);a.add_argument('output',type=Path);a.add_argument('--prior',type=Path,required=True);a.add_argument('--frozen',type=Path);p=a.parse_args();run(p.source,p.output,p.prior,p.frozen)
