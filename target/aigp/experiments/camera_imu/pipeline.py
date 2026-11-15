"""End-to-end measurements. Public inputs are exported before scoring references."""
import json,gzip
from pathlib import Path
import numpy as np


def statistics(values):
    values=np.asarray(values,dtype=float);values=values[np.isfinite(values)]
    if not len(values):return dict(count=0,median=None,p95=None,maximum=None,mean=None)
    return dict(count=len(values),median=float(np.median(values)),p95=float(np.quantile(values,.95)),maximum=float(max(values)),mean=float(np.mean(values)))


def collect(path):
    imu=[];camera=[];sends=[];profiles=[];updates=[];arms=[];observations=[]
    with path.open() as stream:
        for row in map(json.loads,stream):
            event=row.get('event')
            if event=='public_imu':
                imu.extend([[s['time'],s['received_at'],*s['acceleration'],*s['gyro']] for s in row['samples']])
            elif event=='public_camera':camera.append(row)
            elif event=='profile':profiles.append(row)
            elif event=='visual_measurement':observations.append(row)
            elif event=='update':updates.append(row)
            elif event=='call':
                if row['name']=='send' and row['arguments'].get('kind')=='MotorControls':sends.append(row)
                elif row['name']=='arm':arms.append(row)
    values=np.asarray(imu,dtype=float).reshape(-1,8)
    arrays=dict(imu_time=values[:,0],imu_received_at=values[:,1],acceleration=values[:,2:5],gyro=values[:,5:8],
        frame_id=np.array([r['image_id'] for r in camera],dtype=np.int64),
        frame_header_time_ns=np.array([r['header_time_ns'] for r in camera],dtype=np.int64),
        frame_time_estimated_ns=np.array([-1 if r['estimated_time_ns'] is None else r['estimated_time_ns'] for r in camera],dtype=np.int64),
        frame_received_at=np.array([r['received_at'] for r in camera]),
        frame_shape_hw=np.array([r['shape'][:2] for r in camera],dtype=np.int32).reshape(-1,2),
        motor_sent_host_time=np.array([r['host_time'] for r in sends]),
        motor_outputs=np.array([r['arguments']['outputs'] for r in sends]).reshape(-1,4))
    return arrays,dict(profiles=profiles,updates=updates,sends=sends,arms=arms,observations=observations)


def analyze(root):
    arrays,events=collect(root/'flight.jsonl')
    np.savez_compressed(root/'public-inputs.npz',**arrays)
    with gzip.open(root/'public-observations.jsonl.gz','wt') as stream:
        for observed in events['observations']:stream.write(json.dumps(observed,separators=(',',':'))+'\n')
    # Native truth first enters this report here, after the public replay inputs are saved.
    score=json.loads((root/'score.json').read_text());protocol=json.loads((root/'protocol.json').read_text())
    arm=next((r['host_time'] for r in events['arms'] if r['arguments']['armed']),None)
    last=max((r['host_time'] for r in events['profiles'] if r['scope']=='controller'),default=None)
    active=lambda row:arm is not None and row['host_time']>=arm and (last is None or row['host_time']<=last)
    profiles=[r for r in events['profiles'] if active(r)];commands=[r for r in events['sends'] if active(r)]
    stages={}
    with gzip.open(root/'pipeline-timing.csv.gz','wt') as stream:
        stream.write('host_time,scope,stage,wall_ms,thread_cpu_ms,calls\n')
        for row in events['profiles']:
            for name,value in row['stages'].items():
                stream.write(f"{row['host_time']},{row['scope']},{name},{value['wall_ns']/1e6},{value['thread_cpu_ns']/1e6},{value['calls']}\n")
    for row in profiles:
        for name,value in row['stages'].items():stages.setdefault(name,[]).append(value)
    timing={name:dict(wall_ms=statistics([v['wall_ns']/1e6 for v in values]),
        thread_cpu_ms=statistics([v['thread_cpu_ns']/1e6 for v in values]),invocations=sum(v['calls'] for v in values)) for name,values in stages.items()}
    signals=[r['signals'] for r in events['updates'] if r.get('signals')];full={};partial={}
    for state in signals:
        observed=state.get('camera_observation')
        if observed is not None:full.setdefault(observed['image_id'],dict(observed,claimed_gate=state.get('gate_number')))
        observed=state.get('partial_gate')
        if observed is not None:partial.setdefault(observed['image_id'],observed)
    control_times=np.array([r['host_time'] for r in commands]);imu_dt=np.diff(arrays['imu_time'])
    arrival_dt=np.diff(arrays['frame_received_at']);rate_error=[]
    for state in signals:
        if state.get('gyro') is not None and state.get('rate_request') is not None:
            rate_error.append(np.array(state['rate_request'])-state['gyro'])
    rate_error=np.array(rate_error).reshape(-1,3)
    motors=np.array([r['arguments']['outputs'] for r in commands]).reshape(-1,4)
    accepted_frame_ids={f['id'] for row in events['updates'] for f in row.get('frames',[])}
    report=dict(revision=protocol['revision'],input_contract=dict(
        public_imu_samples=len(arrays['imu_time']),decoded_camera_frames=len(arrays['frame_id']),controller_frame_ids=len(accepted_frame_ids),
        unaligned_camera_frames=int(np.sum(arrays['frame_time_estimated_ns']<0)),
        imu_published_interval_ms=statistics(imu_dt[imu_dt>0]*1000),imu_nonincreasing_timestamps=int(np.sum(imu_dt<=0)),
        camera_arrival_interval_ms=statistics(arrival_dt[arrival_dt>0]*1000),
        true_physics_imu_interval_s=None,true_camera_exposure_time_ns=None,
        clock_note='IMU intervals describe published timestamps. Camera times use arrival-envelope alignment, not exposure ground truth.'),
        perception=dict(raw_visual_constraints=len(events['observations']),full_measurements=len(full),partial_measurements=len(partial),
            full_reprojection_px=statistics([o['reprojection_rmse_px'] for o in full.values()]),
            partial_edge_residual_px=statistics([o['edge_residual_px'] for o in partial.values()]),
            association_counts=signals[-1].get('association_counts') if signals else None,
            partial_target_changes=max((s.get('partial_retargets',0) for s in signals),default=0)),
        estimator=dict(scored_local_errors=score.get('local_state_errors'),timing=score.get('estimation'),
            posterior_covariance_available=protocol['settings'].get('joint_estimator',False),
            delayed_position_velocity_repropagation_after_attitude_correction=protocol['settings'].get('joint_estimator',False)),
        planning=dict(timed_trajectory_available=False,uncertainty_constraints_available=False,
            metric_age_s=statistics([s['imu_time']-s['measurement_time'] for s in signals if s.get('measurement_time') is not None]),
            startup_allowance_s=protocol['settings'].get('startup_allowance_s'),speed_limit_m_s=protocol['settings'].get('speed_m_s')),
        control=dict(host_command_interval_ms=statistics(np.diff(control_times)*1000),command_count=len(commands),
            command_intervals_over_25ms=int(np.sum(np.diff(control_times)>.025)),command_intervals_over_40ms=int(np.sum(np.diff(control_times)>.040)),
            rate_tracking_rms_rad_s=None if not len(rate_error) else np.sqrt(np.mean(rate_error**2,axis=0)).tolist(),
            motor_minimum=None if not len(motors) else float(motors.min()),motor_maximum=None if not len(motors) else float(motors.max()),
            motor_sample_saturation_fraction=None if not len(motors) else float(np.mean((motors<=.001)|(motors>=.999)))),
        timing=dict(inclusive_stages=timing,controller_period_goal_ms=20.,host_window=dict(start=arm,end=last),
            note='Nested stage times overlap and must not be summed. Profiling is separate telemetry, excluded from control and exact command comparison.'),
        model=dict(supplied_config=protocol['settings'].get('position_config') is not None,sha256=protocol['settings'].get('dynamics_model_sha256')),
        scoring={k:score.get(k) for k in ['native_gates_passed','estimated_gates_passed','native_completed','native_finish_time_s','valid_race','passed_without_reset','controller_error','reset_calls_during_flight']},
        contacts=len(score.get('in_flight_collision_packets',[])),public_input_export='public-inputs.npz',visual_export='public-observations.jsonl.gz')
    (root/'pipeline.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    from .pipeline_latency import update
    update(root)
    from .pipeline_score import score as score_architecture
    score_architecture(root)
    return json.loads((root/'pipeline.json').read_text())


def render(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    report=json.loads((root/'pipeline.json').read_text());data=np.load(root/'public-inputs.npz',allow_pickle=False)
    fig,ax=plt.subplots(3,2,figsize=(14,11),layout='constrained');fig.suptitle(root.name+' | end-to-end pipeline measurements')
    host=data['motor_sent_host_time'];imu=data['imu_time'];frame=data['frame_received_at']
    if len(host)>1:ax[0,0].plot(host[1:]-host[0],np.diff(host)*1000,'.',ms=2,label='host command interval')
    ax[0,0].axhline(20,color='r',ls='--',label='50Hz goal');ax[0,0].set_xlabel('host elapsed [s]');ax[0,0].set_ylabel('ms');ax[0,0].legend(fontsize=8)
    names=list(report['timing']['inclusive_stages']);p95=[report['timing']['inclusive_stages'][n]['wall_ms']['p95'] for n in names]
    ax[0,1].barh(names,p95);ax[0,1].set_xlabel('p95 wall time [ms; nested stages overlap]')
    if len(imu)>1:ax[1,0].hist(np.diff(imu)*1000,bins=45,alpha=.6,label='published IMU clock interval')
    if len(frame)>1:ax[1,0].hist(np.diff(frame)*1000,bins=45,alpha=.6,label='camera host-arrival interval')
    ax[1,0].set_xlabel('ms');ax[1,0].set_ylabel('samples');ax[1,0].legend(fontsize=8)
    observations=[]
    with gzip.open(root/'public-observations.jsonl.gz','rt') as stream:observations=list(map(json.loads,stream))
    for kind,color in [('aperture','C2'),('outer_top','C4'),('lookahead_aperture','C0')]:
        selected=[o for o in observations if o['kind']==kind]
        ax[1,1].plot([o['capture_time'] for o in selected],[o.get('context_gate',o.get('claimed_gate',0)) for o in selected],'.',ms=2,color=color,label=kind)
    ax[1,1].set_xlabel('estimated IMU-aligned image time [s]');ax[1,1].set_ylabel('mission context, not assigned identity');ax[1,1].legend(fontsize=8)
    errors=root/'state-errors.csv'
    if errors.exists():
        state=np.genfromtxt(errors,delimiter=',',names=True)
        for key,label in [('position_error_m','position [m]'),('velocity_error_m_s','velocity [m/s]'),('attitude_error_deg','attitude [deg]')]:ax[2,0].semilogy(state['time'],np.maximum(state[key],1e-5),label=label)
        ax[2,0].set_xlabel('IMU sample time [s]');ax[2,0].set_ylabel('scored error magnitude');ax[2,0].legend(fontsize=8)
    if len(host):
        for i in range(4):ax[2,1].plot(host-host[0],data['motor_outputs'][:,i],label=f'motor{i+1}')
    ax[2,1].set_ylim(-.02,1.02);ax[2,1].set_xlabel('host elapsed [s]');ax[2,1].set_ylabel('normalized own motor command');ax[2,1].legend(fontsize=8)
    for a in ax.flat:a.grid(alpha=.2)
    output=root/'plots';output.mkdir(exist_ok=True);fig.savefig(output/'pipeline.png',dpi=130);plt.close(fig);data.close()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    if (args.root/'flight.jsonl').exists():analyze(args.root)
    render(args.root)
