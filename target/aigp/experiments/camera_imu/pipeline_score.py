"""Read-only scorer: labels and native geometry never enter a navigator."""
import gzip,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from target.aigp.experiments.camera_imu import vision
from target.aigp.experiments.camera_imu.geometry import gate_openings

def stats(x):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    return dict(count=len(x),median=None if not len(x) else float(np.median(x)),p95=None if not len(x) else float(np.quantile(x,.95)),maximum=None if not len(x) else float(max(x)))

def score(root):
    if not (root/'state-errors.csv').exists():
        result=dict(status='No native approach reference available');(root/'architecture-score.json').write_text(json.dumps(result)+'\n');return result
    native=np.genfromtxt(root/'state-errors.csv',delimiter=',',names=True)
    with gzip.open(root/'control.jsonl.gz','rt') as stream:ticks=list(map(json.loads,stream))
    if native.size<2:
        result=dict(status='Insufficient native approach references');(root/'architecture-score.json').write_text(json.dumps(result)+'\n');return result
    repeated=int(np.sum(np.diff(native['time'])==0));native=native[np.r_[True,np.diff(native['time'])>0]]
    nt=native['time'];rotations=Slerp(nt,Rotation.from_euler('xyz',np.column_stack([native['native_attitude_'+a+'_deg'] for a in ['roll','pitch','yaw']]),degrees=True))
    openings=gate_openings()[:6];center=openings.mean(axis=1)
    associations=[];seen=set();unscored=0
    for tick in ticks:
        o=tick.get('camera_observation')
        if o is None:continue
        key=(o['image_id'],tick['gate_number'])
        if key in seen:continue
        seen.add(key);t=o['capture_time']
        if not nt[0]<=t<=nt[-1]:unscored+=1;continue
        pose=vision.best_pose(np.array(o['corners']))
        if pose is None:unscored+=1;continue
        position=np.array([np.interp(t,nt,native['native_position_'+a]) for a in 'ned'])
        predicted_center=position+rotations(t).as_matrix() @ vision.CAMERA_FROM_BODY.T @ pose[2]
        distances=np.linalg.norm(center-predicted_center,axis=1);indices=np.argsort(distances);nearest=indices[0]
        confident=distances[nearest]<max(3.,.15*np.linalg.norm(pose[2])) and distances[indices[1]]-distances[nearest]>3.
        associations.append(dict(time=t,image_id=int(o['image_id']),assigned_gate=int(tick['gate_number']),scored_gate=int(nearest+1) if confident else None,
                    nearest_center_error_m=float(distances[nearest]),second_distance_gap_m=float(distances[indices[1]]-distances[nearest])))
    time=np.array([t['imu_time'] for t in ticks]);dt=np.diff(time,append=time[-1]);dt=np.maximum(0.,dt)
    phases={key:float(sum(dt[i] for i,t in enumerate(ticks) if t['phase']==key)) for key in sorted({t['phase'] for t in ticks})}
    velocity=np.array([t.get('velocity',[np.nan]*3) for t in ticks]);age=np.array([np.nan if t.get('measurement_time') is None else t['imu_time']-t['measurement_time'] for t in ticks])
    claims=[r for r in associations if r['scored_gate'] is not None];wrong=[r for r in claims if r['scored_gate']!=r['assigned_gate']]
    score=json.loads((root/'score.json').read_text());margins=[]
    for g in score.get('gates',[]):
        if g.get('crossing_offset_width_down_m') is not None:
            width,down=g['crossing_offset_width_down_m'];opening=openings[g['gate']-1]
            halfw=np.linalg.norm(opening[1]-opening[0])/2;halfh=np.linalg.norm(opening[3]-opening[0])/2
            margins.append(dict(gate=g['gate'],width_margin_m=float(halfw-abs(width)),height_margin_m=float(halfh-abs(down))))
    result=dict(source_revision=json.loads((root/'protocol.json').read_text())['revision'],scorer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                scope='Retrospective native scoring only. No labels returned to the pilot. Camera timestamps retain their alignment uncertainty.',repeated_state_times_collapsed=repeated,
                association_rule='Retrospective3D map consistency, not a pixel identity oracle. Nominal-size PnP plus scored native pose chooses nearest map center within max(3m,15%range) and >3m separation from second. Pose/depth error can mimic a wrong landmark. Others unknown.',
                association=dict(claims=len(associations),scored=len(claims),map_inconsistent=len(wrong),unknown=len(associations)-len(claims),outside_state_trace=unscored,inconsistent_frames=wrong),
                planning=dict(phase_duration_imu_s=phases,crossing_alignment_s=float(sum(dt[i] for i,t in enumerate(ticks) if t.get('crossing_alignment'))),visibility_alignment_s=float(sum(dt[i] for i,t in enumerate(ticks) if t.get('visibility_alignment'))),
                        horizontal_speed_below_1m_s_s=float(np.sum(dt*(np.linalg.norm(velocity[:,:2],axis=1)<1.))),
                        usable_measurement_gap_s=stats(age),gate_centerline_crossing_margins=margins,vehicle_body_margin_m=None),
                observation_support=dict(local_position_rms_m=score.get('local_state_errors',{}).get('position_error_m',{}).get('rms'),
                        local_velocity_rms_m_s=score.get('local_state_errors',{}).get('velocity_error_m_s',{}).get('rms'),
                        gate_relative_fused_velocity_rms_m_s=score.get('estimation',{}).get('fused_velocity_m_s',{}).get('rms'),
                        shadow_unfused_velocity_rms_m_s=score.get('estimation',{}).get('unfused_velocity_m_s',{}).get('rms')))
    (root/'architecture-score.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':print(json.dumps(score(Path(sys.argv[1])),indent=2))
