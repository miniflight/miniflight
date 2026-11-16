"""Score frozen public-estimator outputs. Private references are never returned."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .vision import CAMERA_FROM_BODY
from .geometry import gate_openings
from .gate_filter_score import statistics


def interpolate(reference,t):
    if not len(reference):return np.full((len(t),reference.shape[1]-1),np.nan)
    reference=reference[np.r_[True,np.diff(reference[:,0])>0]]
    result=np.column_stack([np.interp(t,reference[:,0],reference[:,j]) for j in range(1,reference.shape[1])])
    result[(t<reference[0,0])|(t>reference[-1,0])]=np.nan
    return result


def score(source,root):
    path=root/'estimates.jsonl.gz';estimate_hash=hashlib.sha256(path.read_bytes()).hexdigest()
    with gzip.open(path,'rt') as h:rows=[json.loads(l) for l in h]
    with gzip.open(source/'control.jsonl.gz','rt') as h:control=[json.loads(l) for l in h]
    fixed=(source/'score-frame.json').exists()
    control=[r for r in control if r['phase'] in ['SHIFTING','TRACKING','FINAL_IMU','HANDOFF']+(['ACQUIRING'] if fixed else []) and r.get('joint_posterior')]
    assert len(rows)==len(control);t=np.array([r['time'] for r in rows])
    if fixed:
        frame=json.loads((source/'score-frame.json').read_text())
        if frame['scale']!=1.:raise ValueError('Scoring map scale must remain fixed')
        yaw=frame['yaw_rad'];align=Rotation.from_euler('z',yaw).as_matrix();origin=np.array(frame['origin_m'])
        with np.load(source/'native-reference.npz',allow_pickle=False) as reference:
            motion=interpolate(reference['position_velocity'],t);a=reference['attitude']
        a=a[np.r_[True,np.diff(a[:,0])>0]];np_=motion[:,:3];nv=motion[:,3:]
        nr=Slerp(a[:,0],Rotation.from_euler('xyz',a[:,1:]))(np.clip(t,a[0,0],a[-1,0])).as_matrix()
    else:
        n=np.genfromtxt(source/'state-errors.csv',delimiter=',',names=True)
        assert len(rows)==len(n);np.testing.assert_array_equal(t,n['time'])
        local0=np.array(control[0]['camera_orientation']) @ CAMERA_FROM_BODY
        world0=Rotation.from_euler('xyz',[n['estimated_attitude_'+a+'_deg'][0] for a in ['roll','pitch','yaw']],degrees=True).as_matrix()
        yaw=Rotation.from_matrix(world0 @ local0.T).as_euler('xyz')[2];align=Rotation.from_euler('z',yaw).as_matrix()
        origin=np.array([n['estimated_position_'+a][0] for a in 'ned'])-align @ control[0]['position']
        np_=np.column_stack([n['native_position_'+a] for a in 'ned']);nv=np.column_stack([n['native_velocity_'+a] for a in 'ned'])
        nr=Rotation.from_euler('xyz',np.column_stack([n['native_attitude_'+a+'_deg'] for a in ['roll','pitch','yaw']]),degrees=True).as_matrix()
    p=np.array([r['position'] for r in rows]) @ align.T+origin;v=np.array([r['velocity'] for r in rows]) @ align.T;R=align @ np.array([r['rotation'] for r in rows])
    pe=p-np_;ve=v-nv;ae=Rotation.from_matrix(np.swapaxes(R,1,2) @ nr).as_rotvec()
    all_openings=gate_openings();centers=all_openings.mean(axis=1);key=np.array([r['gate_number'] for r in control]);truth=centers[key-1]
    map_est=np.full((len(t),3),np.nan);heading=np.full(len(t),np.nan)
    for i,(r,k) in enumerate(zip(rows,key)):
        gate=r['gates'].get(str(k))
        if gate is not None:map_est[i]=align @ gate['center']+origin;heading[i]=gate['yaw']+yaw
    relative=map_est-p;true_relative=truth-np_;re=relative-true_relative;ge=map_est-truth
    camera=np.einsum('ij,nkj,nk->ni',CAMERA_FROM_BODY,R,relative);native_camera=np.einsum('ij,nkj,nk->ni',CAMERA_FROM_BODY,nr,true_relative)
    normal=np.cross(all_openings[:,1]-all_openings[:,0],all_openings[:,3]-all_openings[:,0]);truth_heading=np.arctan2(normal[:,1],normal[:,0])[key-1]
    heading_error=(heading-truth_heading+np.pi/2)%np.pi-np.pi/2
    errors=dict(position=np.linalg.norm(pe,axis=1),velocity=np.linalg.norm(ve,axis=1),attitude=np.degrees(np.linalg.norm(ae,axis=1)),
        gate_center=np.linalg.norm(ge,axis=1),gate_relative=np.linalg.norm(re,axis=1),gate_depth=camera[:,2]-native_camera[:,2],gate_plane_yaw=np.degrees(heading_error))
    private=source/'native-reference.npz'
    private_coverage={}
    if private.exists():
        with np.load(private,allow_pickle=False) as ref:
            angular=interpolate(ref['body_rates'],t);motors=interpolate(ref['motor_outputs'],t)
        errors['body_rate']=np.linalg.norm(np.array([r['body_rates'] for r in rows])-angular,axis=1)
        errors['motor_channel']=np.array([r['motor_outputs'] for r in rows])-motors
        private_coverage=dict(body_rates=float(np.mean(np.isfinite(angular).all(axis=1))),motor_channels=float(np.mean(np.isfinite(motors).all(axis=1))))
    masks=dict(full=np.ones(len(t),bool),startup=np.array([r['phase']=='SHIFTING' for r in control]))
    masks.update({f'gate{k}':key==k for k in np.unique(key)})
    metrics={phase:{k:statistics(np.asarray(x[mask]).ravel()) for k,x in errors.items()} for phase,mask in masks.items()}
    covariance=dict(position=align @ np.array([r['position_covariance'] for r in rows]) @ align.T,
        velocity=align @ np.array([r['velocity_covariance'] for r in rows]) @ align.T,attitude=np.array([r['attitude_covariance'] for r in rows]))
    coverage={k:float(np.mean(abs(e)<=3*np.sqrt(np.maximum(0,np.diagonal(covariance[k],axis1=1,axis2=2))))) for k,e in [('position',pe),('velocity',ve),('attitude',ae)]}
    result=dict(metrics=metrics,coordinate_3sigma_coverage=coverage,private_reference_coverage=private_coverage,estimate_sha256=estimate_hash,
        alignment=dict(yaw_rad=float(yaw),origin_m=origin.tolist(),scale=1.,method='Frozen learned-map frame; no test-start alignment.' if fixed else 'Same fixed initial frame as recorded flight; no candidate-specific alignment or scale fit.'),
        gate_available_fraction=float(np.mean(np.isfinite(map_est[:,0]))),
        definitions=dict(position='Mean3D error norm[m]',velocity='Mean3D error norm[m/s]',attitude='Mean rotation-error angle[deg]',gate_plane_yaw='Plane axis modulo180deg',
            body_rate='Public gyro minus estimated bias versus private ATTITUDE rate channels, FRD rad/s',motor_channel='Mean absolute channel error in raw reported output units; own sent-command model, not measured RPM'),
        reference_use='Private fields enter this scorer only, after estimates are frozen. Nothing returns to estimation.')
    (root/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    assert hashlib.sha256(path.read_bytes()).hexdigest()==estimate_hash
    np.savez_compressed(root/'scored-errors.npz',time=t,position=pe,velocity=ve,attitude=ae,gate_center=ge,gate_relative=re,depth=errors['gate_depth'],gate=key)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels=[('position','Position error[m]'),('velocity','Velocity error[m/s]'),('attitude','Attitude error[deg]'),('gate_depth','Depth error[m]'),('gate_center','Gate-map error[m]'),('gate_relative','Gate-relative error[m]')]
    fig,ax=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for a,(name,label) in zip(ax.flat,labels):
        a.plot(t-t[0],abs(errors[name]));a.set(xlabel='Elapsed public IMU time[s]',ylabel=label);a.grid(alpha=.2)
    fig.suptitle(source.name+' | canonical joint state; private data used only for scoring');fig.savefig(root/'errors.png',dpi=130);plt.close(fig)
    print(json.dumps(metrics['full'],indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('replay',type=Path);a=p.parse_args();score(a.source,a.replay)
