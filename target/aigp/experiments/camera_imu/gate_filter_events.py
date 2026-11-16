"""Exact estimator replay from public inputs; recorded outputs are checks only."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from miniflight.attitude import stationary_rotation
from miniflight.gate_filter import GateFilter,GateHistory
from . import vision

POINTS=.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])
FIELDS=dict(init={'seed','config'},imu={'time','acceleration','gyro'},command={'time','motors'},pixels={'time','gate','image_id','pixels','template'})
CONFIG_FIELDS={'thrust_curve','drag','camera_from_body','intrinsics','supported','attitude_corrections','history_s'}

def state(history):
    f=history.filter
    return dict(time=float(history.time),position=f.p.tolist(),velocity=f.v.tolist(),rotation=f.R.tolist(),
        gyro_bias=f.bg.tolist(),force_bias=f.ba.tolist(),size=float(f.size),
        gates={str(k):dict(index=i,center=g.tolist(),yaw=float(yaw)) for k,(i,g,yaw) in f.gates.items()},
        rim=None if f.rim is None else f.rim[1].tolist(),covariance_sha256=hashlib.sha256(f.P.tobytes()).hexdigest(),
        force_accepted=f.force_accepted,force_rejected=f.force_rejected)

def export(source,root):
    counts=dict(inputs=0,states=0)
    with gzip.open(root/'estimator-inputs.jsonl.gz','wt') as inputs,gzip.open(root/'estimator-states.jsonl.gz','wt') as outputs,source.open() as stream:
        for row in map(json.loads,stream):
            if row.get('event')=='estimator_input':
                kind=row['kind'];allowed={'event','host_time','sequence','kind'}|FIELDS[kind]
                if set(row)!=allowed:raise ValueError('Unexpected estimator input fields')
                inputs.write(json.dumps({k:v for k,v in row.items() if k not in ('event','host_time')},separators=(',',':'),allow_nan=False)+'\n');counts['inputs']+=1
            elif row.get('event')=='estimator_state':
                outputs.write(json.dumps(dict(sequence=row['sequence'],state=row['state']),separators=(',',':'),allow_nan=False)+'\n');counts['states']+=1
    return counts

def replay(source,output,gyro_attitude=None):
    output.mkdir(parents=True)
    expected={}
    with gzip.open(source/'estimator-states.jsonl.gz','rt') as h:
        for row in map(json.loads,h):expected.setdefault(row['sequence'],[]).append(row['state'])
    history=None;serial=0;states=[];changes=[];matched=0;events=0
    with gzip.open(source/'estimator-inputs.jsonl.gz','rt') as stream:
        for row in map(json.loads,stream):
            kind=row['kind']
            if kind not in FIELDS:raise ValueError('Unexpected estimator input kind')
            if set(row)!={'sequence','kind'}|FIELDS[kind]:raise ValueError('Unexpected estimator input fields')
            events+=1
            if row['sequence']!=events:raise ValueError('Estimator input sequence changed')
            before=None if history is None else state(history)
            old_pixels=set() if history is None else set(history.results)
            old_force=None if history is None else history.filter.last_force_time
            if kind=='init':
                if history is not None:raise ValueError('Estimator initialized twice')
                seed=np.array(row['seed']);forces=seed[:,1:4].copy();rates=seed[:,4:7].copy();c=row['config']
                rotation=stationary_rotation(forces.mean(axis=0)).as_rotation_matrix()
                if set(c) not in (CONFIG_FIELDS,CONFIG_FIELDS|{'broad_rim'}):raise ValueError('Unexpected estimator calibration fields')
                f=GateFilter(rotation,np.zeros(3),c['thrust_curve'],c['drag'],c['camera_from_body'],c['intrinsics'],supported=c['supported'])
                f.gyro_noise=max(.001,float(max(rates.std(axis=0))*np.sqrt(np.median(np.diff(seed[:,0])))))
                f.attitude_corrections=c['attitude_corrections'] if gyro_attitude is None else not gyro_attitude
                f.broad_rim=c.get('broad_rim',False)
                history=GateHistory(f,float(seed[-1,0]),seed[-1,4:7],seconds=c['history_s'])
            elif history is None:raise ValueError('Estimator input precedes initialization')
            elif kind=='imu':history.imu(row['time'],row['gyro'],row['acceleration'])
            elif kind=='command':history.command(row['time'],row['motors'])
            else:
                pixels=np.array(row['pixels']);template=row['template'];seed=None
                if template is None:
                    pose=vision.best_pose(pixels)
                    if pose is None:raise ValueError('Recorded admitted corners lost their pose seed')
                    seed=(pose[2],(pose[1][1]-pose[1][0])/1.5)
                history.observe(dict(id=serial,image_id=row['image_id'],time=row['time'],key=row['gate'],pixels=pixels,
                    points=POINTS if template is None else None,seed=seed,rim=template is not None,
                    rim_prior=None if template is None else (template['width_m'],template['center_to_top_m'],template['fractional_spread'])))
                serial+=1
            actual=state(history)
            if before is not None:
                dt=actual['time']-before['time'];dp=np.array(actual['position'])-before['position']
                correction=dp-.5*(np.array(before['velocity'])+actual['velocity'])*dt
                changes.append(dict(sequence=events,kind=kind,time=actual['time'],position_step=dp.tolist(),position_update=correction.tolist(),
                    new_pixels=sorted(set(history.results)-old_pixels),force_result=history.filter.last_force_result if history.filter.last_force_time!=old_force else None))
            for reference in expected.get(events,[]):
                if gyro_attitude is None:
                    if actual!=reference:raise AssertionError(f'Estimation state/covariance differs at input{events}')
                    matched+=1
                states.append(dict(sequence=events,**actual))
    if gyro_attitude is None and matched!=sum(len(v) for v in expected.values()):raise AssertionError('Recorded state has no matching input')
    with gzip.open(output/'estimates.jsonl.gz','wt') as h:
        for r in states:h.write(json.dumps(r,separators=(',',':'))+'\n')
    with gzip.open(output/'corrections.jsonl.gz','wt') as h:
        for r in changes:h.write(json.dumps(r,separators=(',',':'))+'\n')
    with gzip.open(output/'pixel-factors.jsonl.gz','wt') as h:
        for key,value in history.results.items():h.write(json.dumps(dict(id=key,**value),separators=(',',':'))+'\n')
    report=dict(inputs=events,outputs=len(states),matched_live_states=matched,exact_mean_and_covariance=gyro_attitude is None,
        gyro_attitude_override=gyro_attitude,native_inputs=0,late_pixels=history.rejected_late)
    (output/'replay.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--attitude',choices=('recorded','gyro','fusion'),default='recorded');a=p.parse_args()
    replay(a.source,a.output,None if a.attitude=='recorded' else a.attitude=='gyro')
