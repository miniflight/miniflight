"""Causal joint-filter replay from raw public exports; no scoring inputs."""
import argparse,gzip,hashlib,json,time
from bisect import bisect_right
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from common.math import Quaternion
from miniflight.attitude import stationary_rotation
from miniflight.gate_filter import GateFilter,GateHistory
from . import vision

FIELDS={'imu_time','imu_received_at','acceleration','gyro','frame_id','frame_header_time_ns','frame_time_estimated_ns',
        'frame_received_at','frame_shape_hw','motor_sent_host_time','motor_outputs'}
SQUARE=.75*np.array([[0.,-1.,-1.],[0.,1.,-1.],[0.,1.,1.],[0.,-1.,1.]])


def run(source,root,model,tracks=None,force=True):
    root.mkdir(parents=True)
    data_path=source/'public-inputs.npz';image_path=source/'public-observations.jsonl.gz' if tracks is None else tracks
    with np.load(data_path,allow_pickle=False) as raw:
        if set(raw.files)!=FIELDS:raise ValueError('Exactly the raw public-input schema is required')
        data={k:raw[k] for k in raw.files}
    with gzip.open(image_path,'rt') as stream:observations=list(map(json.loads,stream))
    profile=json.loads(model.read_text());config=profile['position_config']
    protocol=dict(source=str(source),inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [data_path,image_path,model]},
        source_code={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path('miniflight/gate_filter.py')]},
        method='Joint p/v/attitude/gyro-bias/force-bias, fixed nominal-size consider parameter, rim dimensions and upright gate xyz/yaw EKF;1s rewind and full event replay. Optional body specific-force observations at20Hz, without acceleration preintegration.',
        association=('Recorded public mission context on raw candidates; this includes candidates the pilot rejected.' if tracks is None else
            'Frozen accepted pixel tracks and own mission IDs. Previous estimated p/v/R are excluded. Availability is conservative send-return time; NIS may reject tracks.'),
        timing='Public timestamp epochs and receipts; published elapsed time is not exact force preintegration. Command epoch uses the preceding public receipt-offset envelope.',
        binding='Three distinct, consistent images can bind a5m-nearer landmark after innovation rejection. The old landmark stays separate; no native identity labels.',
        noise=dict(acceleration_density_m_s2_sqrt_s=.5,gyro_density_floor_rad_sqrt_s=.001,gyro_bias_walk_rad_s_sqrt_s=.0001,force_bias_walk_m_s2_sqrt_s=.05,pixel_sigma_px=1.,
            gyro='Stationary measured density or0.001 floor, plus per-interval angular-rate variation uncertainty.',rim='Two shared dimension states; public P90-P10 spread supplies prior uncertainty.',aperture_size_prior_m=[1.5,.03],
            acceleration_observation_sigma_floor_m_s2=.5,acceleration_observation_period_s=.05,acceleration_timing='Instantaneous body force; uncertainty grows with recent own-command changes.'),
        native_inputs=0,raw_images_retained=False,force_observations_enabled=force,aperture_mean_fixed=True)
    (root/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    events=[]
    for i,t in enumerate(data['imu_received_at']):events.append((t,0,i))
    for i,t in enumerate(data['motor_sent_host_time']):events.append((t,2,i))
    for i,row in enumerate(observations):
        if row['kind'] in ('aperture','outer_top'):events.append((row['host_time'],1,i))
    events.sort();history=None;seed=[];offsets=deque(maxlen=256);outputs=[];timings=[];seen=set();current=None;counter=0
    ignored=dict(uninitialized=0,missing_template=0,missing_gate=0,bad_pose=0,duplicate=0)
    def sample(host):
        f=history.filter;key=current
        value=dict(host_time=host,time=history.time,position=f.p.copy(),velocity=f.v.copy(),rotation=f.R.copy(),
            gyro_bias=f.bg.copy(),force_bias=f.ba.copy(),state_covariance=f.P[:15,:15].copy(),gate=key,
            covariance_min_eigenvalue=float(np.linalg.eigvalsh(f.P).min()),aperture_size_m=f.size,aperture_size_sigma_m=float(np.sqrt(f.P[15,15])))
        if key in f.gates:
            relative,cov,camera,camcov=f.relative(key);index,center,yaw=f.gates[key]
            value.update(gate_center=center.copy(),gate_yaw=yaw,relative_gate=relative,relative_covariance=cov,camera_gate=camera,camera_gate_covariance=camcov,
                position_gate_covariance=f.P[:3,index:index+3].copy())
        outputs.append(value)
    for host,kind,i in events:
        start=time.perf_counter()
        if kind==0:
            stamp=float(data['imu_time'][i]);gyro=data['gyro'][i];acc=data['acceleration'][i];offsets.append(host-stamp)
            if history is None:
                seed.append((stamp,gyro,acc))
                if stamp-seed[0][0]<.5:continue
                forces=np.array([s[2] for s in seed]);rates=np.array([s[1] for s in seed])
                if np.max(np.linalg.norm(rates,axis=1))>.05 or np.max(forces.std(axis=0))>.2:raise RuntimeError('Stationary public initialization failed')
                rotation=stationary_rotation(forces.mean(axis=0)).as_rotation_matrix()
                f=GateFilter(rotation,rates.mean(axis=0),config['thrust_curve'],config['body_resistance_per_m'],vision.CAMERA_FROM_BODY,vision.K)
                dt=np.median(np.diff([s[0] for s in seed]));f.gyro_noise=max(.001,float(max(rates.std(axis=0))*np.sqrt(dt)))
                history=GateHistory(f,stamp,gyro)
            else:history.imu(stamp,gyro,acc if force else None)
        elif history is None:ignored['uninitialized']+=1;continue
        elif kind==2:
            # Record the causal estimate before applying this newly issued own command.
            sample(host);stamp=float(host-min(offsets));history.command(stamp,data['motor_outputs'][i])
        else:
            row=observations[i];key=int(row['context_gate']);current=key
            identity=(row['image_id'],row['kind'],key)
            if identity in seen:ignored['duplicate']+=1;continue
            seen.add(identity);pixels=np.array(row['points']);template=row['quality'].get('template');seed_gate=None
            if row['kind']=='aperture':
                points=SQUARE;pose=vision.best_pose(pixels)
                if pose is None:ignored['bad_pose']+=1;continue
                seed_gate=(pose[2],(pose[1][1]-pose[1][0])/1.5)
            else:
                if template is None:ignored['missing_template']+=1;continue
                w,h=template['width_m'],template['center_to_top_m'];points=np.array([[0.,-w/2,-h],[0.,w/2,-h]])
            history.observe(dict(id=counter,image_id=row['image_id'],time=float(row['capture_time']),key=key,pixels=pixels,points=points,seed=seed_gate,sigma_px=1.,
                rim=row['kind']=='outer_top',rim_prior=None if template is None else (template['width_m'],template['center_to_top_m'],template['fractional_spread'])))
            counter+=1
        timings.append((kind,(time.perf_counter()-start)*1000))
    # Freeze estimates and diagnostics before any scorer is invoked.
    keys=set.intersection(*(set(x) for x in outputs));arrays={k:np.array([r[k] for r in outputs]) for k in keys}
    # A gate may be born after the first command. Preserve its later outputs with an explicit validity mask.
    shapes=dict(gate_center=(3,),gate_yaw=(),relative_gate=(3,),relative_covariance=(3,3),camera_gate=(3,),camera_gate_covariance=(3,3),position_gate_covariance=(3,3))
    for key,shape in shapes.items():arrays[key]=np.array([r.get(key,np.full(shape,np.nan)) for r in outputs])
    arrays['gate_available']=np.array(['camera_gate' in r for r in outputs])
    arrays['gate']=np.array([-1 if r['gate'] is None else r['gate'] for r in outputs],dtype=np.int32)
    np.savez_compressed(root/'estimates.npz',**arrays)
    with gzip.open(root/'estimates.jsonl.gz','wt') as stream:
        for row in outputs:stream.write(json.dumps({k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in row.items()},allow_nan=False)+'\n')
    results=list(history.results.values());accepted=[r for r in results if r['accepted']]
    diagnostics=dict(outputs=len(outputs),image_updates=len(results),accepted=len(accepted),rejected=len(results)-len(accepted),ignored=ignored,
        late_images_rejected=history.rejected_late,repropagated_nodes=history.repropagated,nearer_landmark_bindings=history.filter.retargets,
        acceleration_updates=history.filter.force_accepted,acceleration_rejections=history.filter.force_rejected,
        minimum_covariance_eigenvalue=min(r['covariance_min_eigenvalue'] for r in outputs),
        processing_ms={str(k):dict(median=float(np.median([t for kind,t in timings if kind==k])),p95=float(np.quantile([t for kind,t in timings if kind==k],.95)),maximum=float(max(t for kind,t in timings if kind==k))) for k in [0,1,2]},
        nis=[r for r in results if 'nis' in r])
    (root/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
    print(json.dumps({k:v for k,v in diagnostics.items() if k!='nis'},indent=2));return diagnostics


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--tracks',type=Path)
    parser.add_argument('--no-force',action='store_true',help='Ablate dynamic accelerometer factors; retain stationary gravity initialization')
    parser.add_argument('--model',type=Path,default=Path(__file__).with_name('dynamics.model.json'));args=parser.parse_args()
    run(args.source,args.output,args.model,args.tracks,not args.no_force)
