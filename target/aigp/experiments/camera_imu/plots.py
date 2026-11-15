"""Postflight plots. Native values appear only in scored CSV/JSON inputs."""
import argparse,bisect,csv,gzip,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from target.aigp.experiments.camera_imu.vision import CAMERA_FROM_BODY,K_INV

plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.2,'figure.dpi':130})

def csv_data(path):
    if not path.exists():return {}
    rows=list(csv.DictReader(path.open()))
    return {key:np.array([float(r[key]) if r[key] else np.nan for r in rows]) for key in rows[0]} if rows else {}

def vec(ticks,key):
    return np.array([t.get(key) or [np.nan]*3 for t in ticks],float)

def finish(fig,axes,root,name):
    for ax in np.array(axes).flat:
        ax.set_xlabel('IMU time [s]')
        if ax.lines:ax.legend(fontsize=7,loc='best')
    fig.savefig(root/name);plt.close(fig)

def plot(root):
    ticks=[json.loads(l) for l in gzip.open(root/'control.jsonl.gz','rt')]
    ticks=[t for t in ticks if 'camera_orientation' in t and 'position' in t]
    time=np.array([t['imu_time'] for t in ticks]);position=vec(ticks,'position');velocity=vec(ticks,'velocity')
    gate=np.array([t.get('gate_number',1) for t in ticks]);alignment=np.array([t.get('visibility_alignment',False) for t in ticks])
    age=np.array([np.nan if t.get('full_image_time',t.get('image_time')) is None else t['imu_time']-t.get('full_image_time',t.get('image_time')) for t in ticks]);center=vec(ticks,'gate_center')
    body=np.array([t['camera_orientation'] for t in ticks]) @ CAMERA_FROM_BODY
    euler=Rotation.from_matrix(body).as_euler('xyz',degrees=True)
    errors=csv_data(root/'state-errors.csv');camera=csv_data(root/'camera-errors.csv');score=json.loads((root/'score.json').read_text())
    output=root/'plots';output.mkdir(exist_ok=True)
    requested=json.loads((root/'protocol.json').read_text())['settings']['gates_to_pass']
    title=f"{root.name}: native {score['native_gates_passed']}/{requested}; contacts {len(score['in_flight_collision_packets'])}; clean {score['passed_without_reset']}"
    fig,ax=plt.subplots(4,2,figsize=(14,12),layout='constrained');fig.suptitle(title)
    for i,name in enumerate(['north','east','down']):
        ax[0,0].plot(time,position[:,i],label=f'estimated {name}')
        ax[0,1].plot(time,velocity[:,i],label=f'estimated {name}')
        ax[1,0].plot(time,euler[:,i],label=['roll','pitch','yaw'][i])
    ax[0,0].set_ylabel('local position [m]');ax[0,1].set_ylabel('velocity [m/s]');ax[1,0].set_ylabel('body attitude [deg]')
    ax[1,1].step(time,gate,where='post',label='estimated current gate')
    crossing=[g for g in score['gates'] if g.get('crossing_imu_time_s') is not None]
    ax[1,1].step([time[0]]+[g['crossing_imu_time_s'] for g in crossing],[0]+[g['gate'] for g in crossing],where='post',label='native geometric crossings [scorer]');ax[1,1].set_ylabel('gate count / index')
    for axis,key,label in [(ax[2,0],'position_error_m','position error [m]'),(ax[2,1],'velocity_error_m_s','velocity error [m/s]'),(ax[3,0],'attitude_error_deg','attitude error [deg]')]:
        axis.plot(errors['time'],errors[key],label='scored error');axis.set_ylabel(label)
    ax[3,1].plot(time,age,label='full gate image age');ax[3,1].plot(time,alignment.astype(float),label='height alignment active');ax[3,1].axhline(4.5,color='r',linestyle='--',label='blind limit');ax[3,1].set_ylabel('age [s] / active [0,1]')
    finish(fig,ax,output,'state.png')
    fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained');fig.suptitle(title+' | camera')
    if camera:
        ct=camera['capture_time']
        for key,label in [('true_range_m','native range [scorer]'),('measured_range_m','camera range')]:ax[0,0].plot(ct,camera[key],'.',ms=2,label=label)
        ax[0,0].set_ylabel('range [m]')
        ax[0,1].plot(ct,camera['position_error_m'],'.',ms=2,label='camera position error');ax[0,1].set_ylabel('error [m]')
        ax[1,0].plot(ct,camera['reprojection_rmse_px'],'.',ms=2,label='image reprojection RMS');ax[1,0].set_ylabel('pixels')
        ax[1,1].plot(ct,camera['capture_to_output_ms'],'.',ms=2,label='estimated capture-to-output age');ax[1,1].plot(ct,camera['velocity_fit_age_ms'],'.',ms=2,label='velocity fit effective age');ax[1,1].set_ylabel('ms')
        ax[2,0].plot(ct,camera['velocity_error_m_s'],'.',ms=2,label='camera velocity error');ax[2,0].set_ylabel('m/s')
    seen=set();obs=[]
    for t in ticks:
        o=t.get('camera_observation')
        if o and o['image_id'] not in seen:
            seen.add(o['image_id']);corners=np.array(o['corners']);obs.append([o['capture_time'],o['minimum_side_px'],float(np.min(np.r_[corners[:,0],639-corners[:,0],corners[:,1],359-corners[:,1]]))])
    obs=np.array(obs)
    if len(obs):
        ax[2,1].plot(obs[:,0],obs[:,1],label='minimum aperture side');ax[2,1].plot(obs[:,0],obs[:,2],label='aperture border margin');ax[2,1].set_ylabel('pixels')
    finish(fig,ax,output,'camera.png')
    if 'native_position_n' in errors:
        fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained');fig.suptitle(title+' | estimates versus native scoring reference')
        for i,direction in enumerate('ned'):
            for name,style in [('estimated','-'),('native','--')]:
                ax[i,0].plot(errors['time'],errors[f'{name}_position_{direction}'],style,label=name)
                ax[i,1].plot(errors['time'],errors[f'{name}_velocity_{direction}'],style,label=name)
            ax[i,0].set_ylabel(f'{direction.upper()} position [m]');ax[i,1].set_ylabel(f'{direction.upper()} velocity [m/s]')
        finish(fig,ax,output,'reference.png')
        fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained');fig.suptitle(title+' | attitude and body rates')
        requested=vec(ticks,'rate_request');gyro=vec(ticks,'gyro')
        for i,direction in enumerate(['roll','pitch','yaw']):
            for name,style in [('estimated','-'),('native','--')]:ax[i,0].plot(errors['time'],errors[f'{name}_attitude_{direction}_deg'],style,label=name)
            ax[i,1].plot(time,np.degrees(requested[:,i]),label='requested rate');ax[i,1].plot(time,np.degrees(gyro[:,i]),label='public gyro')
            ax[i,0].set_ylabel(f'{direction} [deg]');ax[i,1].set_ylabel(f'{direction} rate [deg/s]')
        finish(fig,ax,output,'attitude-rates.png')
    motors=np.array([t.get('motor_commands') or [np.nan]*4 for t in ticks]);finite=np.isfinite(motors).all(axis=1)
    saturation=None
    if finite.any():
        saturation=dict(threshold=.001,motor_sample_fraction=float(np.mean((motors[finite]<=.001)|(motors[finite]>=.999))),
                        any_motor_tick_fraction=float(np.mean(np.any((motors[finite]<=.001)|(motors[finite]>=.999),axis=1))))
        fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained');fig.suptitle(title+' | motor and rate control')
        for i in range(4):ax[0,0].plot(time,motors[:,i],label=f'motor {i+1}')
        ax[0,0].set_ylim(-.03,1.03);ax[0,0].set_ylabel('normalized motor command')
        ax[0,1].plot(time,np.min(motors,axis=1),label='minimum');ax[0,1].plot(time,np.max(motors,axis=1),label='maximum');ax[0,1].set_ylim(-.03,1.03);ax[0,1].set_ylabel('motor command envelope')
        requested=vec(ticks,'rate_request');gyro=vec(ticks,'gyro');integral=vec(ticks,'rate_integral')
        for i,name in enumerate(['roll','pitch','yaw']):
            ax[1,0].plot(time,np.degrees(requested[:,i]-gyro[:,i]),label=name)
            ax[1,1].plot(time,integral[:,i],label=name)
        ax[1,0].set_ylabel('rate tracking error [deg/s]');ax[1,1].set_ylabel('PI integral [rad]')
        step=np.array([t.get('alignment_height_step',np.nan) if t.get('visibility_alignment') else np.nan for t in ticks])
        ax[2,0].plot(time,step,label='actual recovery height step');ax[2,0].set_ylabel('m, down+')
        o=[t['camera_observation'] for t in ticks if t.get('camera_observation')]
        if o:
            ot=[a['capture_time'] for a in o]
            for key,label in [('position_correction','position [m]'),('velocity_correction','velocity [m/s]')]:ax[2,1].plot(ot,[np.linalg.norm(a[key]) for a in o],'.',ms=2,label=label)
        ax[2,1].set_ylabel('camera corrections')
        finish(fig,ax,output,'control.png')
    rows=[]
    for t in ticks:
        h=t.get('alignment_hint') or t.get('search_hint')
        if not h:continue
        i=max(0,bisect.bisect_right(time,h['time'])-1)
        ray=np.array(ticks[i]['camera_orientation']) @ K_INV @ [*h['pixel'],1.]
        rows.append([t['imu_time'],t.get('gate_number',1),h['pixel'][1],np.clip((h['pixel'][1]-220)/40,-3,3),np.clip(8*ray[2]/max(np.linalg.norm(ray[:2]),1e-6),-3,3),t.get('visibility_alignment',False),t['imu_time']-t.get('image_time',t['imu_time']),t['imu_time']-h['time']])
    cue=np.array(rows)
    transitions=[]
    for g in range(2,7):
        selected=(gate==g)&alignment
        if not selected.any():continue
        lo=time[selected].min()-.5;hi=time[selected].max()+.5;sel=(time>=lo)&(time<=hi)
        fig,ax=plt.subplots(4,2,figsize=(14,12),layout='constrained');fig.suptitle(title+f' | gate {g} recovery')
        ax[0,0].plot(time[sel],position[sel,2],label='estimated down position');ax[0,0].plot(time[sel],center[sel,2],label='estimated gate down position');ax[0,0].set_ylabel('local down [m]')
        rel=vec(ticks,'alignment_relative_gate');ax[0,1].plot(time[sel],rel[sel,2],label='estimated relative height');ax[0,1].axhline(0,color='k',linestyle='--');ax[0,1].set_ylabel('gate minus drone [m, down+]')
        ax[1,0].plot(time[sel],velocity[sel,2],label='estimated down velocity');ax[1,0].set_ylabel('m/s')
        ax[1,1].plot(time[sel],euler[sel,1],label='pitch');ax[1,1].set_ylabel('deg')
        ax[2,0].plot(time[sel],age[sel],label='full gate image age');ax[2,0].axhline(4.5,color='r',linestyle='--',label='blind limit');ax[2,0].set_ylabel('s')
        ax[3,0].plot(time[sel],np.linalg.norm(velocity[sel,:2],axis=1),label='estimated horizontal speed');ax[3,0].plot(time[sel],np.linalg.norm((center-position)[sel,:2],axis=1),label='estimated horizontal range');ax[3,0].set_ylabel('m/s or m')
        if len(cue):
            c=cue[(cue[:,0]>=lo)&(cue[:,0]<=hi)]
            if len(c):
                ax[2,1].plot(c[:,0],c[:,3],label='old pixel-row height step');ax[2,1].plot(c[:,0],c[:,4],label='world-bearing height step');ax[2,1].set_ylabel('bounded vertical step [m]')
                ax[3,1].plot(c[:,0],c[:,2],label='partial-red centroid row');ax[3,1].axhline(220,color='k',linestyle='--',label='old image target row');ax[3,1].axhline(359,color='r',linestyle='--',label='image bottom');ax[3,1].set_ylabel('pixels, down+')
        finish(fig,ax,output,f'gate-{g}.png')
        transitions.append(dict(gate=g,alignment_duration_s=float(np.sum(np.clip(np.diff(time,append=time[-1]),0,.1)*selected)),
            maximum_full_pose_age_s=float(np.nanmax(age[selected])) if np.isfinite(age[selected]).any() else None,
            full_pose_missing_ticks=int(np.sum(~np.isfinite(age[selected]))),
            horizontal_speed_at_end_m_s=float(np.linalg.norm(velocity[selected][-1,:2])),relative_height_at_end_m=float(rel[selected][-1,2])))
    metrics=dict(name=root.name,native_gates=score['native_gates_passed'],clean=score['passed_without_reset'],contacts=len(score['in_flight_collision_packets']),finish_s=score['native_finish_time_s'],local_state_errors=score['local_state_errors'],recovery=transitions,motor_saturation=saturation,missing_motor_trace=saturation is None,plots=[p.name for p in sorted(output.glob('*.png'))])
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(json.dumps(dict(name=root.name,plots=len(metrics['plots']),recovery=transitions)))
    return metrics

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('roots',nargs='+',type=Path);args=parser.parse_args()
    for root in args.roots:plot(root)
